/* Capacity reuse, failure atomicity and secret clearing, with real QR frames.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_LINEAR
#define STBI_NO_HDR
#include "k_quirc_internal.h"
#include "stb_image.h"

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      fprintf(stderr, "line %d: %s\n", __LINE__, #condition);                  \
      exit(1);                                                                 \
    }                                                                          \
  } while (0)

static struct {
  void *ptr;
  size_t size;
  bool must_scrub;
} allocations[8];
static int allocation_calls;
static int fail_call;

static void check_zero(const void *ptr, size_t size) {
  const uint8_t *bytes = ptr;
  for (size_t i = 0; i < size; i++)
    CHECK(bytes[i] == 0);
}

static void *image_alloc(size_t size) {
  if (++allocation_calls == fail_call)
    return NULL;
  for (size_t i = 0; i < 8; i++) {
    if (!allocations[i].ptr) {
      void *ptr = malloc(size);
      CHECK(ptr);
      allocations[i].ptr = ptr;
      allocations[i].size = size;
      allocations[i].must_scrub = false;
      return ptr;
    }
  }
  CHECK(false);
  return NULL;
}

static void checked_free(void *ptr) {
  for (size_t i = 0; i < 8; i++) {
    if (ptr && allocations[i].ptr == ptr) {
      if (allocations[i].must_scrub)
        check_zero(ptr, allocations[i].size);
      allocations[i].ptr = NULL;
      break;
    }
  }
  free(ptr);
}

#undef K_MALLOC_IMAGE
#undef K_FREE
#define K_MALLOC_IMAGE(size) image_alloc(size)
#define K_FREE(ptr) checked_free(ptr)
#include "../src/k_quirc.c"

/* Poison the entire allocation, including inactive capacity, to verify that
 * growth/destruction scrub capacity rather than just the current dimensions. */
static void poison_images(k_quirc_t *q) {
  for (size_t i = 0; i < 8; i++) {
    if (allocations[i].ptr == q->image || allocations[i].ptr == q->pixels) {
      memset(allocations[i].ptr, 0xa5, allocations[i].size);
      allocations[i].must_scrub = true;
    }
  }
}

static void test_capacity(void) {
  k_quirc_t *q = k_quirc_new();
  CHECK(q);
  CHECK(k_quirc_resize(q, 600, 600) == 0);
  uint8_t *image = q->image;
  quirc_pixel_t *pixels = q->pixels;
  uint8_t *stack = q->flood_fill_stack;
  int calls = allocation_calls;
  k_quirc_set_threshold_offset_for(q, 23);
  poison_images(q);

  /* The next allocation will fail, but shrinking and returning to full frame
   * must still work. A different aspect ratio also fits the same capacity. */
  fail_call = calls + 1;
  const int sizes[][2] = {{320, 320}, {600, 600}, {600, 600}, {720, 500},
                          {64, 64},   {600, 600}, {320, 320}};
  for (size_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
    size_t old_size = (size_t)q->w * q->h;
    size_t size = (size_t)sizes[i][0] * sizes[i][1];
    CHECK(k_quirc_resize(q, sizes[i][0], sizes[i][1]) == 0);
    int w, h;
    CHECK(k_quirc_begin(q, &w, &h) == image);
    CHECK(w == sizes[i][0] && h == sizes[i][1]);
    CHECK(q->pixels == pixels && q->flood_fill_stack == stack);
    CHECK(q->image_capacity == 600 * 600);
    CHECK(allocation_calls == calls);
    CHECK(k_quirc_get_threshold_offset_for(q) == 23);
    if (size < old_size) {
      check_zero(image + size, old_size - size);
      if (q->owns_pixels)
        check_zero(pixels + size, (old_size - size) * sizeof(*pixels));
    }
    memset(image, 0xa5, size);
    if (q->owns_pixels)
      memset(pixels, 0xa5, size * sizeof(*pixels));
  }

  CHECK(k_quirc_resize(q, 0, 320) == -1);
  CHECK(k_quirc_resize(q, -1, 320) == -1);
  CHECK(k_quirc_resize(q, K_QUIRC_MAX_IMAGE_DIM + 1, 1) == -1);
  CHECK(allocation_calls == calls);
  CHECK(k_quirc_resize(q, 640, 640) == -1);
  CHECK(q->image == image && q->pixels == pixels);
  CHECK(q->w == 320 && q->h == 320 && q->image[0] == 0xa5);
  CHECK(q->image_capacity == 600 * 600);

  if (q->owns_pixels) {
    fail_call = allocation_calls + 2; /* Image succeeds, labels fail. */
    CHECK(k_quirc_resize(q, 640, 640) == -1);
    CHECK(q->image == image && q->pixels == pixels);
    CHECK(q->w == 320 && q->h == 320 && q->image[0] == 0xa5);
    CHECK(q->image_capacity == 600 * 600);
  }

  fail_call = 0;
  poison_images(q);
  CHECK(k_quirc_resize(q, 640, 640) == 0);
  CHECK(q->image_capacity == 640 * 640);
  CHECK(k_quirc_get_threshold_offset_for(q) == 23);
  CHECK(k_quirc_resize(q, 64, 64) == 0);
  poison_images(q);
  k_quirc_destroy(q);
}

static void decode_frame(k_quirc_t *q, const uint8_t *src, int sw, int sh,
                         int w, int h, k_quirc_result_t *result) {
  CHECK(k_quirc_resize(q, w, h) == 0);
  /* Hold threshold constant to compare reused storage with a fresh decoder. */
  k_quirc_set_threshold_offset_for(q, K_QUIRC_THRESHOLD_OFFSET_DEFAULT);
  uint8_t *buf = k_quirc_begin(q, NULL, NULL);
  CHECK(buf);
  memset(buf, 255, (size_t)w * h);
  for (int y = 0; y < sh; y++)
    memcpy(buf + (size_t)(y + (h - sh) / 2) * w + (w - sw) / 2,
           src + (size_t)y * sw, sw);
  k_quirc_end(q, false);
  CHECK(k_quirc_count(q) == 1);
  CHECK(k_quirc_decode(q, 0, result) == K_QUIRC_SUCCESS);
  CHECK(result->valid && result->data.payload_len > 0);
}

static void test_decode_transitions(void) {
  int sw, sh, channels;
  uint8_t *src = stbi_load(RESIZE_VECTOR, &sw, &sh, &channels, 1);
  CHECK(src && sw <= 600 && sh <= 600);
  k_quirc_t *q = k_quirc_new();
  CHECK(q && k_quirc_resize(q, 600, 600) == 0);
  for (int i = 0; i < 8; i++) {
    int w = i % 2 ? sw : 600;
    int h = i % 2 ? sh : 600;
    int calls = allocation_calls;
    k_quirc_result_t reused, fresh;
    decode_frame(q, src, sw, sh, w, h, &reused);
    CHECK(allocation_calls == calls);
    k_quirc_t *baseline = k_quirc_new();
    CHECK(baseline);
    decode_frame(baseline, src, sw, sh, w, h, &fresh);
    CHECK(memcmp(&reused, &fresh, sizeof(fresh)) == 0);
    k_quirc_destroy(baseline);
  }
  k_quirc_destroy(q);
  stbi_image_free(src);
}

int main(void) {
  test_capacity();
  test_decode_transitions();
  for (size_t i = 0; i < 8; i++)
    CHECK(!allocations[i].ptr);
  puts("Capacity reuse, secure clearing and decode transitions passed");
  return 0;
}
