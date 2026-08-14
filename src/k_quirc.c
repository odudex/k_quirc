/*
 * K-Quirc - Standalone QR Code Recognition Library
 * Public API implementation
 *
 * Original Copyright (C) 2010-2012 Daniel Beer <dlbeer@gmail.com>
 * OpenMV modifications Copyright (c) 2013-2021 Ibrahim Abdelkader
 * <iabdalkader@openmv.io> OpenMV modifications Copyright (c) 2013-2021 Kwabena
 * W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for
 * details.
 */

#include "k_quirc_internal.h"

/* Indirecting through a volatile pointer stops the compiler from treating the
 * zeroing as a dead store and eliding it. See k_quirc_internal.h. */
static void *(*const volatile k_quirc_memset_fn)(void *, int, size_t) = memset;

void k_quirc_bzero(void *ptr, size_t len) {
  if (ptr && len)
    k_quirc_memset_fn(ptr, 0, len);
}

/* Bytes backing q->image / q->pixels for the context's current dimensions. */
static size_t image_bytes(const k_quirc_t *q, size_t elem_size) {
  if (!q || q->w <= 0 || q->h <= 0)
    return 0;
  return (size_t)q->w * (size_t)q->h * elem_size;
}

static int image_allocation_size(int w, int h, size_t elem_size,
                                 size_t *out_size) {
  if (!out_size || w <= 0 || h <= 0 || w > K_QUIRC_MAX_IMAGE_DIM ||
      h > K_QUIRC_MAX_IMAGE_DIM)
    return -1;

  size_t width = (size_t)w;
  size_t height = (size_t)h;
  if (width > SIZE_MAX / height)
    return -1;

  size_t pixels = width * height;
  if (elem_size && pixels > SIZE_MAX / elem_size)
    return -1;

  *out_size = pixels * elem_size;
  return 0;
}

k_quirc_t *k_quirc_new(void) {
  k_quirc_t *q = K_MALLOC_CONTEXT(sizeof(*q));
  if (q) {
    memset(q, 0, sizeof(*q));
#ifdef K_QUIRC_ADAPTIVE_THRESHOLD
    q->threshold_offset = k_quirc_get_threshold_offset();
#endif
  }
  return q;
}

void k_quirc_destroy(k_quirc_t *q) {
  if (q) {
    /* Scrub decoded plaintext and the captured frame before handing the memory
     * back to the allocator. This runs once per scan session, not per frame -
     * callers bracket the whole camera loop with k_quirc_new()/_destroy() and
     * use k_quirc_begin()/_end() per frame - so the cost is immaterial. */
    if (q->image) {
      k_quirc_bzero(q->image, image_bytes(q, sizeof(*q->image)));
      K_FREE(q->image);
    }
    if (q->owns_pixels && q->pixels) {
      k_quirc_bzero(q->pixels, image_bytes(q, sizeof(*q->pixels)));
      K_FREE(q->pixels);
    }
    /* flood_fill_stack holds only coordinates, never payload bytes. */
    if (q->flood_fill_stack)
      K_FREE(q->flood_fill_stack);
    k_quirc_bzero(&q->code_scratch, sizeof(q->code_scratch));
    k_quirc_bzero(&q->data_scratch, sizeof(q->data_scratch));
    k_quirc_bzero(&q->ds_scratch, sizeof(q->ds_scratch));
    K_FREE(q);
  }
}

int k_quirc_resize(k_quirc_t *q, int w, int h) {
  size_t image_size;
  uint8_t *new_image;
  quirc_pixel_t *new_pixels = NULL;
  uint8_t *new_stack = NULL;

  if (!q || image_allocation_size(w, h, sizeof(uint8_t), &image_size) < 0)
    return -1;

  new_image = K_MALLOC_IMAGE(image_size);
  if (!new_image)
    return -1;

  if (sizeof(*q->image) != sizeof(*q->pixels)) {
    size_t pixel_size;
    if (image_allocation_size(w, h, sizeof(quirc_pixel_t), &pixel_size) < 0) {
      K_FREE(new_image);
      return -1;
    }
    new_pixels = K_MALLOC_IMAGE(pixel_size);
    if (!new_pixels) {
      K_FREE(new_image);
      return -1;
    }
  }

  if (!q->flood_fill_stack) {
    new_stack = K_MALLOC_SCRATCH(QUIRC_FLOOD_FILL_STACK * sizeof(xylf_t));
    if (!new_stack) {
      if (new_pixels)
        K_FREE(new_pixels);
      K_FREE(new_image);
      return -1;
    }
  }

  /* q->w/q->h still describe the outgoing buffers here - they are updated
   * below - so this scrubs the previous frame at its own dimensions. */
  if (q->image) {
    k_quirc_bzero(q->image, image_bytes(q, sizeof(*q->image)));
    K_FREE(q->image);
  }
  if (q->owns_pixels && q->pixels) {
    k_quirc_bzero(q->pixels, image_bytes(q, sizeof(*q->pixels)));
    K_FREE(q->pixels);
  }

  q->image = new_image;
  if (sizeof(*q->image) == sizeof(*q->pixels)) {
    q->pixels = (quirc_pixel_t *)q->image;
    q->owns_pixels = false;
  } else {
    q->pixels = new_pixels;
    q->owns_pixels = true;
  }
  if (new_stack)
    q->flood_fill_stack = new_stack;
  q->w = w;
  q->h = h;

  return 0;
}

uint8_t *k_quirc_begin(k_quirc_t *q, int *w, int *h) {
  if (!q || !q->image) {
    if (w)
      *w = 0;
    if (h)
      *h = 0;
    return NULL;
  }

  q->num_regions = QUIRC_PIXEL_REGION;
  q->num_capstones = 0;
  q->num_grids = 0;
  q->flood_fill_overflow = false;

  if (w)
    *w = q->w;
  if (h)
    *h = q->h;

  return q->image;
}

void k_quirc_end(k_quirc_t *q, bool find_inverted) {
  if (!q || !q->image || !q->pixels || !q->flood_fill_stack)
    return;

  k_quirc_identify(q, find_inverted);
}

int k_quirc_count(const k_quirc_t *q) { return q ? q->num_grids : 0; }

k_quirc_error_t k_quirc_decode(k_quirc_t *q, int index,
                               k_quirc_result_t *result) {
  if (!result)
    return K_QUIRC_ERROR_INVALID_GRID_SIZE;

  memset(result, 0, sizeof(*result));

  if (!q || index < 0 || index >= q->num_grids)
    return K_QUIRC_ERROR_INVALID_GRID_SIZE;

  struct quirc_code *code = &q->code_scratch;
  struct quirc_data *data = &q->data_scratch;
  struct datastream *ds = &q->ds_scratch;

  quirc_extract_internal(q, index, code);

  /* Report the detected QR's corner positions regardless of decode outcome.
   * quirc_extract_internal fills code->corners before decoding is attempted, so
   * a located-but-undecoded code can still be measured by the caller (e.g. to
   * compute pixels-per-module). result->valid stays false on decode failure. */
  for (int i = 0; i < 4; i++) {
    result->corners[i].x = code->corners[i].x;
    result->corners[i].y = code->corners[i].y;
  }

  k_quirc_error_t err = quirc_decode_internal(code, data, ds);

  /* Data-ECC failures are frequently a slightly misfitted grid rather than a
   * bad image: the corner opposite the corner capstone is extrapolated (v1
   * codes have no alignment pattern at all), and at small pixels-per-module
   * a sub-module error there flips enough cells to exceed the ECC budget.
   * Retry the extraction with that corner nudged around its fitted position.
   * This costs nothing on the happy path and only re-runs the (cheap)
   * extract+decode stages on frames that would otherwise be dropped.
   * Restricted to small grids (<= v10): larger versions have alignment
   * patterns anchoring the far corner so the nudge rarely helps there,
   * and extraction cost scales with grid area. */
  if (err == K_QUIRC_ERROR_DATA_ECC && q->grids[index].grid_size <= 57) {
    static const float nudges[][2] = {
        {-0.5f, -0.5f}, {0.5f, 0.5f}, {-0.5f, 0.0f},  {0.5f, 0.0f},
        {0.0f, -0.5f},  {0.0f, 0.5f}, {-1.0f, -1.0f}, {1.0f, 1.0f},
        {-1.0f, 0.0f},  {1.0f, 0.0f}, {0.0f, -1.0f},  {0.0f, 1.0f},
    };
    for (size_t n = 0; n < sizeof(nudges) / sizeof(nudges[0]); n++) {
      quirc_extract_nudged(q, index, code, nudges[n][0], nudges[n][1]);
      err = quirc_decode_internal(code, data, ds);
      if (err == K_QUIRC_SUCCESS)
        break;
      err = K_QUIRC_ERROR_DATA_ECC;
    }
  }

  if (err == K_QUIRC_SUCCESS) {
    result->valid = true;
    result->data.version = data->version;
    result->data.ecc_level = data->ecc_level;
    result->data.mask = data->mask;
    result->data.data_type = data->data_type;
    result->data.payload_len = data->payload_len;
    if (result->data.payload_len >= K_QUIRC_MAX_PAYLOAD)
      result->data.payload_len = K_QUIRC_MAX_PAYLOAD - 1;
    result->data.eci = data->eci;
    memcpy(result->data.payload, data->payload, result->data.payload_len);
    result->data.payload[result->data.payload_len] = 0;
  }

  /* The payload now lives in the caller's result, which it is responsible for
   * clearing. Drop our copies rather than leaving them resident in the context
   * for the remainder of the scan session. quirc_decode_internal() zeroes both
   * on entry, so this only shortens the window - it changes no behaviour. */
  k_quirc_bzero(data, sizeof(*data));
  k_quirc_bzero(ds, sizeof(*ds));

  return err;
}

const char *k_quirc_strerror(k_quirc_error_t err) {
  static const char *error_table[] = {
      [K_QUIRC_SUCCESS] = "Success",
      [K_QUIRC_ERROR_INVALID_GRID_SIZE] = "Invalid grid size",
      [K_QUIRC_ERROR_INVALID_VERSION] = "Invalid version",
      [K_QUIRC_ERROR_FORMAT_ECC] = "Format data ECC failure",
      [K_QUIRC_ERROR_DATA_ECC] = "ECC failure",
      [K_QUIRC_ERROR_UNKNOWN_DATA_TYPE] = "Unknown data type",
      [K_QUIRC_ERROR_DATA_OVERFLOW] = "Data overflow",
      [K_QUIRC_ERROR_DATA_UNDERFLOW] = "Data underflow",
      [K_QUIRC_ERROR_ALLOC_FAILED] = "Memory allocation failed",
      [K_QUIRC_ERROR_INVALID_SYMBOL] = "Invalid symbol for data type"};

  if (err >= 0 && err < sizeof(error_table) / sizeof(error_table[0]))
    return error_table[err];

  return "Unknown error";
}

int k_quirc_decode_grayscale(const uint8_t *grayscale_data, int width,
                             int height, k_quirc_result_t *results,
                             int max_results, bool find_inverted) {
  size_t image_size;
  if (!grayscale_data || max_results <= 0 || !results ||
      image_allocation_size(width, height, sizeof(uint8_t), &image_size) < 0)
    return 0;

  k_quirc_t *q = k_quirc_new();
  if (!q)
    return 0;

  if (k_quirc_resize(q, width, height) < 0) {
    k_quirc_destroy(q);
    return 0;
  }

  uint8_t *buf = k_quirc_begin(q, NULL, NULL);
  if (!buf) {
    k_quirc_destroy(q);
    return 0;
  }
  memcpy(buf, grayscale_data, image_size);

  k_quirc_end(q, find_inverted);

  int count = k_quirc_count(q);
  int decoded = 0;

  for (int i = 0; i < count && decoded < max_results; i++) {
    k_quirc_error_t err = k_quirc_decode(q, i, &results[decoded]);
    if (err == K_QUIRC_SUCCESS) {
      decoded++;
    }
  }

  k_quirc_destroy(q);
  return decoded;
}

#ifdef K_QUIRC_DEBUG

static k_quirc_debug_info_t debug_info;

const k_quirc_debug_info_t *k_quirc_get_debug_info(const k_quirc_t *q) {
  if (!q)
    return NULL;

  debug_info.pixels = q->pixels;
  debug_info.w = q->w;
  debug_info.h = q->h;
  debug_info.num_grids = q->num_grids;

  for (int i = 0; i < q->num_grids && i < K_QUIRC_DEBUG_MAX_GRIDS; i++) {
    memcpy(debug_info.grids[i].c, q->grids[i].c, sizeof(float) * 8);
    debug_info.grids[i].grid_size = q->grids[i].grid_size;
  }

  debug_info.num_capstones = q->num_capstones;
  for (int i = 0; i < q->num_capstones && i < K_QUIRC_DEBUG_MAX_CAPSTONES;
       i++) {
    debug_info.capstones[i].x = q->capstones[i].center.x;
    debug_info.capstones[i].y = q->capstones[i].center.y;
  }

  debug_info.threshold_offset = k_quirc_get_threshold_offset_for(q);

  return &debug_info;
}

#endif /* K_QUIRC_DEBUG */
