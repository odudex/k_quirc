/*
 * K-Quirc Identification Module
 * QR code detection: flood-fill, thresholding, capstone and grid detection
 */

#include "k_quirc_internal.h"
#include <limits.h>

#define TAG "k_quirc"

/*
 * LIFO (stack) for flood-fill — uses persistent buffer from struct k_quirc
 */
typedef struct {
  xylf_t *data;
  size_t len;
  size_t capacity;
} lifo_t;

/*
 * Pixel-row scanning primitives.  The fast paths process 4 pixels per
 * iteration; they require byte-sized pixels and a little-endian target
 * (both ESP32 and typical hosts), otherwise the plain loops are used.
 */
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define K_QUIRC_LE_WORD_SCAN 1
#else
#define K_QUIRC_LE_WORD_SCAN 0
#endif

/* Index of the lowest / highest non-zero byte of v != 0.  The ESP32-P4 has no
 * count-zeros instruction, so __builtin_ctz/clz would be library calls. */
ALWAYS_INLINE int low_byte(uint32_t v) {
  return (v & 0xffffu) ? ((v & 0xffu) ? 0 : 1) : ((v & 0xff0000u) ? 2 : 3);
}

ALWAYS_INLINE int high_byte(uint32_t v) {
  return (v >> 16) ? ((v >> 24) ? 3 : 2) : ((v >> 8) ? 1 : 0);
}

/* First index in [x, w) where row[index] != color; w if none.
 * The word loop uses memcpy loads, so alignment is never violated; both
 * x86 and the ESP32-P4 handle the unaligned accesses in hardware. */
ALWAYS_INLINE int row_run_end(const quirc_pixel_t *row, int x, int w,
                              quirc_pixel_t color) {
  if (K_QUIRC_LE_WORD_SCAN && sizeof(quirc_pixel_t) == 1) {
    uint32_t pat = (uint32_t)color * 0x01010101u;
    while (x + 4 <= w) {
      uint32_t v;
      memcpy(&v, row + x, 4);
      v ^= pat;
      if (v)
        return x + low_byte(v);
      x += 4;
    }
  }
  while (x < w && row[x] == color)
    x++;
  return x;
}

/* First index of the run of `color` pixels ending just before `left`:
 * scans row[left-1], row[left-2], ... while they equal color. */
ALWAYS_INLINE int row_run_start(const quirc_pixel_t *row, int left,
                                quirc_pixel_t color) {
  if (K_QUIRC_LE_WORD_SCAN && sizeof(quirc_pixel_t) == 1) {
    uint32_t pat = (uint32_t)color * 0x01010101u;
    while (left >= 4) {
      uint32_t v;
      memcpy(&v, row + left - 4, 4);
      v ^= pat;
      if (v)
        return left - 3 + high_byte(v);
      left -= 4;
    }
  }
  while (left > 0 && row[left - 1] == color)
    left--;
  return left;
}

/* First index in [x, limit] where row[index] == color; -1 if none.
 * Open-coded zero-byte scan: most spans are short, so the call overhead
 * of libc memchr would dominate. */
ALWAYS_INLINE int row_find_pixel(const quirc_pixel_t *row, int x, int limit,
                                 quirc_pixel_t color) {
  if (K_QUIRC_LE_WORD_SCAN && sizeof(quirc_pixel_t) == 1) {
    uint32_t pat = (uint32_t)color * 0x01010101u;
    while (x + 4 <= limit + 1) {
      uint32_t v;
      memcpy(&v, row + x, 4);
      v ^= pat;
      uint32_t zero = (v - 0x01010101u) & ~v & 0x80808080u;
      if (zero)
        return x + low_byte(zero);
      x += 4;
    }
  }
  for (; x <= limit; x++)
    if (row[x] == color)
      return x;
  return -1;
}

/* Last index in [limit, x] where row[index] == color; -1 if none.  The zero
 * test is the exact one: the borrow trick can flag the byte above a zero. */
ALWAYS_INLINE int row_find_pixel_back(const quirc_pixel_t *row, int x,
                                      int limit, quirc_pixel_t color) {
  if (K_QUIRC_LE_WORD_SCAN && sizeof(quirc_pixel_t) == 1) {
    uint32_t pat = (uint32_t)color * 0x01010101u;
    while (x - 3 >= limit) {
      uint32_t v;
      memcpy(&v, row + x - 3, 4);
      v ^= pat;
      uint32_t zero = ~(((v & 0x7f7f7f7fu) + 0x7f7f7f7fu) | v | 0x7f7f7f7fu);
      if (zero)
        return x - 3 + high_byte(zero);
      x -= 4;
    }
  }
  for (; x >= limit; x--)
    if (row[x] == color)
      return x;
  return -1;
}

ALWAYS_INLINE void fill_span(quirc_pixel_t *p, int len, quirc_pixel_t v) {
  if (sizeof(quirc_pixel_t) == 1) {
    memset(p, v, (size_t)len);
  } else {
    for (int i = 0; i < len; i++)
      p[i] = v;
  }
}

/*
 * Linear algebra routines
 */
static K_QUIRC_WARN_UNUSED_RESULT int
line_intersect(const struct quirc_point *p0, const struct quirc_point *p1,
               const struct quirc_point *q0, const struct quirc_point *q1,
               struct quirc_point *r) {
  int a = -(p1->y - p0->y);
  int b = p1->x - p0->x;
  int c = -(q1->y - q0->y);
  int d = q1->x - q0->x;
  int e = a * p1->x + b * p1->y;
  int f = c * q1->x + d * q1->y;
  int det = (a * d) - (b * c);

  if (!det)
    return 0;

  r->x = (d * e - b * f) / det;
  r->y = (-c * e + a * f) / det;
  return 1;
}

static K_QUIRC_WARN_UNUSED_RESULT int
perspective_setup(float *c, const struct quirc_point *rect, float w, float h) {
  float x0 = rect[0].x;
  float y0 = rect[0].y;
  float x1 = rect[1].x;
  float y1 = rect[1].y;
  float x2 = rect[2].x;
  float y2 = rect[2].y;
  float x3 = rect[3].x;
  float y3 = rect[3].y;

  float wden = w * (x2 * y3 - x3 * y2 + (x3 - x2) * y1 + x1 * (y2 - y3));
  float hden = h * (x2 * y3 + x1 * (y2 - y3) - x3 * y2 + (x3 - x2) * y1);

  if (fabsf(wden) < 1e-6f || fabsf(hden) < 1e-6f) {
    memset(c, 0, sizeof(float) * QUIRC_PERSPECTIVE_PARAMS);
    return 0;
  }

  c[0] = (x1 * (x2 * y3 - x3 * y2) +
          x0 * (-x2 * y3 + x3 * y2 + (x2 - x3) * y1) + x1 * (x3 - x2) * y0) /
         wden;
  c[1] = -(x0 * (x2 * y3 + x1 * (y2 - y3) - x2 * y1) - x1 * x3 * y2 +
           x2 * x3 * y1 + (x1 * x3 - x2 * x3) * y0) /
         hden;
  c[2] = x0;
  c[3] = (y0 * (x1 * (y3 - y2) - x2 * y3 + x3 * y2) + y1 * (x2 * y3 - x3 * y2) +
          x0 * y1 * (y2 - y3)) /
         wden;
  c[4] = (x0 * (y1 * y3 - y2 * y3) + x1 * y2 * y3 - x2 * y1 * y3 +
          y0 * (x3 * y2 - x1 * y2 + (x2 - x3) * y1)) /
         hden;
  c[5] = y0;
  c[6] = (x1 * (y3 - y2) + x0 * (y2 - y3) + (x2 - x3) * y1 + (x3 - x2) * y0) /
         wden;
  c[7] = (-x2 * y3 + x1 * y3 + x3 * y2 + x0 * (y1 - y2) - x3 * y1 +
          (x2 - x1) * y0) /
         hden;
  return 1;
}

static int solve_8x8_system(float A[8][8], float b[8], float x[8]) {
  for (int k = 0; k < 8; k++) {
    int max_row = k;
    float max_val = fabsf(A[k][k]);
    for (int i = k + 1; i < 8; i++) {
      if (fabsf(A[i][k]) > max_val) {
        max_val = fabsf(A[i][k]);
        max_row = i;
      }
    }

    if (max_row != k) {
      for (int j = k; j < 8; j++) {
        float tmp = A[k][j];
        A[k][j] = A[max_row][j];
        A[max_row][j] = tmp;
      }
      float tmp = b[k];
      b[k] = b[max_row];
      b[max_row] = tmp;
    }

    float pivot = A[k][k];
    if (fabsf(pivot) < 1e-10f) {
      for (int i = 0; i < 8; i++)
        x[i] = 0.0f;
      return 0;
    }

    for (int i = k + 1; i < 8; i++) {
      float factor = A[i][k] / pivot;
      for (int j = k; j < 8; j++) {
        A[i][j] -= factor * A[k][j];
      }
      b[i] -= factor * b[k];
    }
  }

  for (int i = 7; i >= 0; i--) {
    x[i] = b[i];
    for (int j = i + 1; j < 8; j++) {
      x[i] -= A[i][j] * x[j];
    }
    x[i] /= A[i][i];
  }
  return 1;
}

static K_QUIRC_WARN_UNUSED_RESULT int
perspective_setup_direct(float *c, const float img[4][2],
                         const float mod[4][2]) {
  float A[8][8];
  float b[8];

  for (int i = 0; i < 4; i++) {
    float u = mod[i][0], v = mod[i][1];
    float x = img[i][0], y = img[i][1];
    int row1 = i * 2, row2 = i * 2 + 1;

    A[row1][0] = u;
    A[row1][1] = v;
    A[row1][2] = 1.0f;
    A[row1][3] = 0.0f;
    A[row1][4] = 0.0f;
    A[row1][5] = 0.0f;
    A[row1][6] = -u * x;
    A[row1][7] = -v * x;
    b[row1] = x;

    A[row2][0] = 0.0f;
    A[row2][1] = 0.0f;
    A[row2][2] = 0.0f;
    A[row2][3] = u;
    A[row2][4] = v;
    A[row2][5] = 1.0f;
    A[row2][6] = -u * y;
    A[row2][7] = -v * y;
    b[row2] = y;
  }

  return solve_8x8_system(A, b, c);
}

/*
 * Span-based flood fill from reg->seed, recording the region's area and
 * bounding box.  Out of stack, the area is left at 0.
 */
HOT_FUNC
static void flood_fill_seed(struct k_quirc *q, struct quirc_region *reg,
                            quirc_pixel_t from_color, quirc_pixel_t to_color) {
  int x = reg->seed.x;
  int y = reg->seed.y;

  lifo_t lifo;
  lifo.data = (xylf_t *)q->flood_fill_stack;
  lifo.len = 0;
  lifo.capacity = QUIRC_FLOOD_FILL_STACK;

  reg->count = 0;
  reg->x0 = reg->x1 = (int16_t)x;
  reg->y0 = reg->y1 = (int16_t)y;

  /* Neighbor-row scan state for the active span.  Pixels only ever change
   * away from from_color, so after returning from a child span the scan can
   * resume where it stopped instead of rescanning from `left`.  The state
   * is packed into the stack slot the original algorithm used for the
   * (otherwise unused on resume) seed x: scan >= 0 means "scanning the row
   * above, next index scan"; scan < 0 means "scanning the row below, next
   * index -scan-1". */
  int scan;
  int left, right;

  for (;;) {
    {
      quirc_pixel_t *row = q->pixels + y * q->w;

      left = row_run_start(row, x, from_color);
      right = row_run_end(row, x + 1, q->w, from_color) - 1;

      fill_span(row + left, right - left + 1, to_color);

      reg->count += right - left + 1;
      if (left < reg->x0)
        reg->x0 = (int16_t)left;
      if (right > reg->x1)
        reg->x1 = (int16_t)right;
      if (y < reg->y0)
        reg->y0 = (int16_t)y;
      if (y > reg->y1)
        reg->y1 = (int16_t)y;

      scan = left; /* start with the row above */
    }

    for (;;) {
      /* What to come back to, should a neighbour row need filling */
      bool recurse = false;
      xylf_t context = {0, (int16_t)y, (int16_t)left, (int16_t)right};

      if (lifo.len >= lifo.capacity) {
        reg->count = 0; /* out of stack */
        return;
      }

      if (scan >= 0) {
        if (y > 0) {
          const quirc_pixel_t *row = q->pixels + (y - 1) * q->w;
          int i = row_find_pixel(row, scan, right, from_color);
          if (i >= 0) {
            context.x = (int16_t)(i + 1);
            x = i;
            y = y - 1;
            recurse = true;
          }
        }
        if (!recurse)
          scan = -left - 1; /* row above done; switch to the row below */
      }

      if (!recurse && scan < 0 && y < q->h - 1) {
        const quirc_pixel_t *row = q->pixels + (y + 1) * q->w;
        int i = row_find_pixel(row, -scan - 1, right, from_color);
        if (i >= 0) {
          context.x = (int16_t)(-i - 2);
          x = i;
          y = y + 1;
          recurse = true;
        }
      }

      if (recurse) {
        lifo.data[lifo.len++] = context;
        break;
      }

      if (!lifo.len)
        return;

      context = lifo.data[--lifo.len];
      scan = context.x;
      y = context.y;
      left = context.l;
      right = context.r;
    }
  }
}

/*
 * Thresholding with Otsu's method
 */
static uint8_t otsu_threshold(uint32_t *histogram, uint32_t total) {
  float sum = 0;
  for (int i = 0; i < 256; i++) {
    sum += (float)i * histogram[i];
  }

  float sumB = 0;
  uint32_t wB = 0;
  float varMax = -1.0f;
  /* First and last level of the maximum plateau.  A strictly bimodal image
   * leaves an empty gap between its two levels, and every threshold inside
   * that gap yields the same partition and therefore the same between-class
   * variance.  Keeping only the first (or only the last) level would place
   * the threshold flush against one of the two modes, so a small adaptive
   * offset is enough to swallow that mode entirely.  Return the midpoint of
   * the plateau instead, which is the usual Otsu convention. */
  int lo = 0;
  int hi = 0;

  for (int i = 0; i < 256; i++) {
    wB += histogram[i];
    if (wB == 0)
      continue;

    uint32_t wF = total - wB;
    if (wF == 0)
      break;

    sumB += (float)i * histogram[i];
    float mB = sumB / wB;
    float mF = (sum - sumB) / wF;
    float mDiff = mB - mF;

    float varBetween = (float)wB * (float)wF * mDiff * mDiff;
    if (varBetween > varMax) {
      varMax = varBetween;
      lo = i;
      hi = i;
    } else if (varBetween == varMax) {
      /* Empty histogram bins leave wB, wF and sumB untouched, so the
       * comparison is exact for the gap of a bimodal image. */
      hi = i;
    }
  }

  return (uint8_t)((lo + hi) / 2);
}

// Percentage of image border to ignore for threshold calculation (0.0 - 0.5)
// 0.2 = ignore 20% on each edge, leaving 60% central region
#define K_QUIRC_THRESHOLD_MARGIN 0.2f

#ifdef K_QUIRC_ADAPTIVE_THRESHOLD
static int default_threshold_offset = K_QUIRC_THRESHOLD_OFFSET_DEFAULT;
#endif

static inline int clamp_threshold(int t) {
  return (t < 0) ? 0 : (t > 255) ? 255 : t;
}

#ifdef K_QUIRC_PIE
void k_quirc_binarize_pie(uint8_t *pixels, int blocks, const uint8_t *t,
                          int t_stride);
#define K_QUIRC_BINARIZE_ALIGN 15
#else
#define K_QUIRC_BINARIZE_ALIGN 3
#endif

/* Binarize a span against a constant threshold, t in 0..255, four pixels at
 * a time.  With H the top bit of every byte, (v | H) - (t & ~H) cannot borrow
 * between bytes and leaves in each top bit whether the pixel's low seven bits
 * reach the threshold's; the two top bits settle the rest. */
ALWAYS_INLINE void binarize_span(quirc_pixel_t *p, int len, int t,
                                 uint8_t xor_mask) {
  int i = 0;

  if (K_QUIRC_LE_WORD_SCAN && sizeof(quirc_pixel_t) == 1) {
    /* Pixel by pixel up to a boundary: the vector unit needs one, and an
     * unaligned word load and store costs the ESP32-P4 a third more */
    for (; i < len && ((uintptr_t)(p + i) & K_QUIRC_BINARIZE_ALIGN); i++)
      p[i] = ((p[i] ^ xor_mask) < t) ? QUIRC_PIXEL_BLACK : QUIRC_PIXEL_WHITE;

#ifdef K_QUIRC_PIE
    if (!xor_mask && len - i >= 16) {
      uint8_t t8 = (uint8_t)t;
      k_quirc_binarize_pie(p + i, (len - i) / 16, &t8, 0);
      i += (len - i) & ~15;
    }
#endif

    const uint32_t H = 0x80808080u;
    const uint32_t mask = xor_mask * 0x01010101u;
    const uint32_t t_low = (uint32_t)(t & 0x7f) * 0x01010101u;
    const uint32_t t_high = (t & 0x80) ? ~(uint32_t)0 : 0;

    for (; i + 4 <= len; i += 4) {
      uint32_t v;
      memcpy(&v, p + i, 4);
      v ^= mask;
      uint32_t reaches = (v | H) - t_low;
      uint32_t below = (~v & ~reaches) | (t_high & ~(v & reaches));
      v = (below & H) >> 7;
      memcpy(p + i, &v, 4);
    }
  }
  for (; i < len; i++)
    p[i] = ((p[i] ^ xor_mask) < t) ? QUIRC_PIXEL_BLACK : QUIRC_PIXEL_WHITE;
}

/* Histogram of every other pixel of every other row, which places Otsu's
 * threshold just as well.  Returns the number of samples. */
static uint32_t histogram_rect(const quirc_pixel_t *pixels, int stride, int x0,
                               int y0, int x1, int y1, uint32_t *hist) {
  uint32_t samples = 0;

  memset(hist, 0, 256 * sizeof(uint32_t));
  for (int y = y0; y < y1; y += 2) {
    const quirc_pixel_t *row = pixels + y * stride;
    for (int x = x0; x < x1; x += 2)
      hist[row[x]]++;
    samples += (uint32_t)(x1 - x0 + 1) / 2;
  }
  return samples;
}

#ifdef K_QUIRC_BILINEAR_THRESHOLD
/*
 * The threshold follows the image's white level, measured per block of a mesh
 * and interpolated between the blocks' centres.
 *
 * A screen read at an angle through an uncorrected lens can show its far
 * corner at half the white level of its centre, and that corner is the one
 * without a finder.  Otsu's threshold of the centre, which sets the operating
 * point, is therefore scaled by how the white level found locally compares
 * with the centre's.  Under even light that is the centre's threshold
 * everywhere.
 */
#define THRESHOLD_MAX_GRID 8
#define THRESHOLD_MIN_BLOCK 64
#define THRESHOLD_BINS 64
#define THRESHOLD_BIN_SHIFT 2
#define THRESHOLD_SAMPLE_STEP 3
#define THRESHOLD_SAME_LIGHT 2
#define THRESHOLD_CELL 16
#define THRESHOLD_MAX_CELLS (K_QUIRC_MAX_IMAGE_DIM / THRESHOLD_CELL + 2)

static int threshold_grid(int dim) {
  int blocks = dim / THRESHOLD_MIN_BLOCK;
  return blocks < 1                    ? 1
         : blocks > THRESHOLD_MAX_GRID ? THRESHOLD_MAX_GRID
                                       : blocks;
}

/* The bin a 64th of a histogram's samples lie beyond, from the top or from the
 * bottom.  Blur dims a lone white module but not the widest white a region
 * holds, so the top one is its white level whatever the size of its features,
 * quiet zone and dense data alike. */
static int histogram_tail(const uint32_t *hist, int bins, uint32_t total,
                          bool top) {
  uint32_t beyond = 0;
  int i = top ? bins - 1 : 0;
  int last = top ? 0 : bins - 1;
  int step = top ? -1 : 1;

  while (i != last && (beyond += hist[i]) * 64 < total)
    i += step;
  return i;
}

/* Where along a mesh of `blocks` blocks of `size` a position falls: the block
 * whose centre precedes it, and how far towards the next it is, of 256.
 * Constant beyond the outermost centres. */
static void threshold_mesh_at(int pos, int size, int blocks, int *block,
                              int *weight) {
  int at = (pos * 256 - size * 128) / size;

  if (at < 0)
    at = 0;
  if (at > (blocks - 1) * 256)
    at = (blocks - 1) * 256;
  *block = at >> 8;
  *weight = at & 255;
}

/* Binarize a row whose threshold is t[k] over its k-th run of 16 pixels.  The
 * runs are laid on 16-byte boundaries of memory, where the vector unit works,
 * so they start up to 15 pixels into the row; the threshold hardly changes
 * over that. */
ALWAYS_INLINE void binarize_row(quirc_pixel_t *row, int w, const uint8_t *t) {
  int head = (int)(-(uintptr_t)row & (THRESHOLD_CELL - 1));

  if (head > w)
    head = w;
  int blocks = (w - head) / THRESHOLD_CELL;
  int tail = head + blocks * THRESHOLD_CELL;
  const uint8_t *cell = t + (head + THRESHOLD_CELL / 2) / THRESHOLD_CELL;

  binarize_span(row, head, t[0], 0);
#ifdef K_QUIRC_PIE
  if (blocks && sizeof(quirc_pixel_t) == 1) {
    k_quirc_binarize_pie(row + head, blocks, cell, 1);
  } else
#endif
  {
    for (int b = 0; b < blocks; b++)
      binarize_span(row + head + b * THRESHOLD_CELL, THRESHOLD_CELL, cell[b],
                    0);
  }
  binarize_span(row + tail, w - tail, cell[blocks], 0);
}

/* Binarize against `t`, the threshold where the white level is bin `white`,
 * scaled by the white level found around each pixel; `offset` is added after.
 */
HOT_FUNC
static void binarize_by_white_level(struct k_quirc *q, int t, int white,
                                    int offset) {
  int w = q->w;
  int h = q->h;
  quirc_pixel_t *pixels = q->pixels;
  int gx = threshold_grid(w);
  int gy = threshold_grid(h);
  int bw = (w + gx - 1) / gx;
  int bh = (h + gy - 1) / gy;
  int16_t t_block[THRESHOLD_MAX_GRID * THRESHOLD_MAX_GRID];
  int16_t t_local[THRESHOLD_MAX_GRID * THRESHOLD_MAX_GRID];

  /* A row of blocks at a time: its histograms fit the stack and the cache */
  for (int by = 0; by < gy; by++) {
    uint16_t hist[THRESHOLD_MAX_GRID][THRESHOLD_BINS];
    int y1 = (by + 1) * bh < h ? (by + 1) * bh : h;

    memset(hist, 0, sizeof(hist));
    for (int y = by * bh; y < y1; y += THRESHOLD_SAMPLE_STEP) {
      const quirc_pixel_t *row = pixels + y * w;
      for (int bx = 0; bx < gx; bx++) {
        uint16_t *block = hist[bx];
        int x1 = (bx + 1) * bw < w ? (bx + 1) * bw : w;
        for (int x = bx * bw; x < x1; x += THRESHOLD_SAMPLE_STEP)
          block[row[x] >> THRESHOLD_BIN_SHIFT]++;
      }
    }

    for (int bx = 0; bx < gx; bx++) {
      uint32_t wide[THRESHOLD_BINS];
      uint32_t total = 0;
      for (int i = 0; i < THRESHOLD_BINS; i++) {
        wide[i] = hist[bx][i];
        total += wide[i];
      }
      int local = histogram_tail(wide, THRESHOLD_BINS, total, true);
      int dark = histogram_tail(wide, THRESHOLD_BINS, total, false);
      int level = t;
      /* Without both classes a block is background or quiet zone, whose level
       * says nothing of the light on the code.  Within two bins of the
       * centre's it is the same light, measured twice.  Only dimmer is
       * followed: the centre's threshold already clears anything brighter. */
      if (2 * dark > local)
        level = -1;
      else if (white - local > THRESHOLD_SAME_LIGHT)
        level = t * (2 * local + 1) / (2 * white + 1);
      t_local[by * gx + bx] = (int16_t)level;
    }
  }

  /* Those take after the blocks around them, or failing any, the centre */
  for (int by = 0; by < gy; by++) {
    for (int bx = 0; bx < gx; bx++) {
      int sum = 0;
      int n = 0;

      t_block[by * gx + bx] = t_local[by * gx + bx];
      if (t_local[by * gx + bx] >= 0)
        continue;
      for (int ny = by - 1; ny <= by + 1; ny++) {
        for (int nx = bx - 1; nx <= bx + 1; nx++) {
          if (nx < 0 || ny < 0 || nx >= gx || ny >= gy ||
              t_local[ny * gx + nx] < 0)
            continue;
          sum += t_local[ny * gx + nx];
          n++;
        }
      }
      t_block[by * gx + bx] = (int16_t)(n ? sum / n : t);
    }
  }

  /* Each 16-pixel cell of a row lies between the same two columns of blocks
   * whatever the row */
  uint8_t cell_block[THRESHOLD_MAX_CELLS];
  uint8_t cell_weight[THRESHOLD_MAX_CELLS];
  uint8_t cell_t[THRESHOLD_MAX_CELLS];
  int cells = w / THRESHOLD_CELL + 2;

  for (int k = 0; k < cells; k++) {
    int block, weight;
    threshold_mesh_at(k * THRESHOLD_CELL + THRESHOLD_CELL / 2, bw, gx, &block,
                      &weight);
    cell_block[k] = (uint8_t)block;
    cell_weight[k] = (uint8_t)weight;
  }

  for (int y = 0; y < h; y++) {
    /* The thresholds move a level or two in four rows */
    if ((y & 3) == 0) {
      int column[THRESHOLD_MAX_GRID + 1];
      int block, weight;

      threshold_mesh_at(y + 2, bh, gy, &block, &weight);
      const int16_t *upper = t_block + block * gx;
      const int16_t *lower =
          t_block + (block + 1 < gy ? block + 1 : block) * gx;
      for (int bx = 0; bx < gx; bx++)
        column[bx] = upper[bx] * (256 - weight) + lower[bx] * weight;
      column[gx] = column[gx - 1];

      for (int k = 0; k < cells; k++) {
        int b = cell_block[k];
        int level = (column[b] * (256 - cell_weight[k]) +
                     column[b + 1] * cell_weight[k]) >>
                    16;
        cell_t[k] = (uint8_t)clamp_threshold(level + offset);
      }
    }
    binarize_row(pixels + y * w, w, cell_t);
  }
}
#endif /* K_QUIRC_BILINEAR_THRESHOLD */

HOT_FUNC
static void threshold(struct k_quirc *q, bool inverted) {
  int w = q->w;
  int h = q->h;
  int margin_x = (int)(w * K_QUIRC_THRESHOLD_MARGIN);
  int margin_y = (int)(h * K_QUIRC_THRESHOLD_MARGIN);
  uint32_t histogram[256];
  uint32_t sampled_pixels = histogram_rect(
      q->pixels, w, margin_x, margin_y, w - margin_x, h - margin_y, histogram);
  int t = otsu_threshold(histogram, sampled_pixels);
  int offset = 0;

#ifdef K_QUIRC_ADAPTIVE_THRESHOLD
  offset = q->threshold_offset;
#endif

#ifdef K_QUIRC_BILINEAR_THRESHOLD
  /* An inverted image's light is its ink, and ink has no level to follow */
  if (!inverted) {
    int white = histogram_tail(histogram, 256, sampled_pixels, true);
    binarize_by_white_level(q, t, white >> THRESHOLD_BIN_SHIFT, offset);
    return;
  }
#endif

  /* XOR mask unifies inverted/non-inverted into a single comparison.
   * Normal: pixel < threshold = black. Inverted: (pixel^0xFF) < threshold. */
  binarize_span(q->pixels, w * h, clamp_threshold(t + offset),
                inverted ? 0xFF : 0x00);
}

HOT_FUNC
static int region_code(struct k_quirc *q, int x, int y) {
  int pixel;
  struct quirc_region *box;
  int region;

  if (x < 0 || y < 0 || x >= q->w || y >= q->h)
    return -1;

  pixel = q->pixels[y * q->w + x];

  if (pixel >= QUIRC_PIXEL_REGION)
    return pixel;

  if (pixel == QUIRC_PIXEL_WHITE)
    return -1;

  if (q->num_regions >= QUIRC_MAX_REGIONS)
    return -1;

  region = q->num_regions;
  box = &q->regions[q->num_regions++];

  memset(box, 0, sizeof(*box));

  box->seed.x = x;
  box->seed.y = y;
  box->capstone = -1;

  flood_fill_seed(q, box, pixel, region);
  return box->count ? region : -1;
}

/* A corner search: the best score and every point that reached it.  Blur
 * flattens a corner into a short diagonal whose points tie on a projection;
 * the corner is their centre rather than whichever came first.  Distances can
 * tie between two different corners, so there the first point stands. */
struct corner_score {
  int best;
  int ties;
  struct quirc_point sum;
};

static void corner_offer(struct corner_score *c, int score, int x, int y,
                         bool average) {
  if (score < c->best || (score == c->best && !average))
    return;
  if (score > c->best) {
    c->best = score;
    c->ties = 0;
    c->sum.x = 0;
    c->sum.y = 0;
  }
  c->ties++;
  c->sum.x += x;
  c->sum.y += y;
}

static void corner_take(const struct corner_score *c, struct quirc_point *p) {
  if (!c->ties)
    return;
  p->x = (c->sum.x + c->ties / 2) / c->ties;
  p->y = (c->sum.y + c->ties / 2) / c->ties;
}

/* Offer both ends of every row of a region.  What a corner maximizes - a
 * distance, a projection - is convex along a row, so nothing between a row's
 * outermost pixels can beat them, and no flood fill is needed to visit them.
 * With `far`, scores are distances from ref; otherwise projections on the
 * axis ref and its normal, one per corner. */
static void region_corners(const struct k_quirc *q, int rcode,
                           const struct quirc_point *ref, bool far,
                           struct corner_score *corners) {
  const struct quirc_region *reg = &q->regions[rcode];

  for (int y = reg->y0; y <= reg->y1; y++) {
    const quirc_pixel_t *row = q->pixels + y * q->w;
    int xs[2];
    xs[0] = row_find_pixel(row, reg->x0, reg->x1, (quirc_pixel_t)rcode);
    if (xs[0] < 0)
      continue;
    xs[1] = row_find_pixel_back(row, reg->x1, xs[0], (quirc_pixel_t)rcode);

    for (int i = 0; i < 2; i++) {
      if (far) {
        int dx = xs[i] - ref->x;
        int dy = y - ref->y;
        corner_offer(&corners[0], dx * dx + dy * dy, xs[i], y, false);
      } else {
        int up = xs[i] * ref->x + y * ref->y;
        int rt = xs[i] * -ref->y + y * ref->x;
        corner_offer(&corners[0], up, xs[i], y, true);
        corner_offer(&corners[1], rt, xs[i], y, true);
        corner_offer(&corners[2], -up, xs[i], y, true);
        corner_offer(&corners[3], -rt, xs[i], y, true);
      }
    }
  }
}

static void find_region_corners(struct k_quirc *q, int rcode,
                                const struct quirc_point *ref,
                                struct quirc_point *corners) {
  struct corner_score score[4] = {{.best = -1}};
  struct quirc_point far = *ref;

  /* The point farthest from the reference, which is inside the stone, is a
   * corner; the axis through the two picks out all four. */
  region_corners(q, rcode, ref, true, score);
  corner_take(&score[0], &far);
  far.x -= ref->x;
  far.y -= ref->y;

  for (int i = 0; i < 4; i++) {
    score[i].best = INT_MIN;
    score[i].ties = 0;
  }
  region_corners(q, rcode, &far, false, score);
  for (int i = 0; i < 4; i++)
    corner_take(&score[i], &corners[i]);
}

#ifdef K_QUIRC_ADAPTIVE_THRESHOLD
/*
 * Measure how far binarization has moved the black/white boundary, using the
 * finder pattern as a built-in calibration target.
 *
 * A finder is 7x7 modules of known composition: a 3x3 black stone, a
 * one-module white ring, and a one-module black ring.  If binarization
 * dilates black by d modules, then in units of module area
 *
 *     white = 16 - 32d          ring = 24 + 48d
 *
 * Both are exactly linear -- the white ring is squeezed from both sides while
 * the outer ring grows on both, and the quadratic terms cancel -- so
 * eliminating the unknown module area between them leaves
 *
 *     d = (2*ring - 3*white) / (4*ring + 6*white)
 *
 * independent of scale, focus and viewing angle.  Counted per capstone during
 * detection, so a single finder steers the next frame even when no grid forms.
 */
static void accumulate_dilation(struct k_quirc *q, int ring_code,
                                const struct quirc_point *corners) {
  int x0 = corners[0].x, x1 = corners[0].x;
  int y0 = corners[0].y, y1 = corners[0].y;

  for (int i = 1; i < 4; i++) {
    if (corners[i].x < x0)
      x0 = corners[i].x;
    if (corners[i].x > x1)
      x1 = corners[i].x;
    if (corners[i].y < y0)
      y0 = corners[i].y;
    if (corners[i].y > y1)
      y1 = corners[i].y;
  }
  if (x0 < 0)
    x0 = 0;
  if (y0 < 0)
    y0 = 0;
  if (x1 >= q->w)
    x1 = q->w - 1;
  if (y1 >= q->h)
    y1 = q->h - 1;

  const quirc_pixel_t code = (quirc_pixel_t)ring_code;
  uint32_t ring = 0;
  uint32_t white = 0;

  /* The measure is a ratio of areas, so some thirty rows serve as well as
   * all of them; a large finder has hundreds. */
  int step = (y1 - y0) / 32 + 1;

  for (int y = y0; y <= y1; y += step) {
    const quirc_pixel_t *row = q->pixels + (size_t)y * q->w;
    uint32_t pending = 0;
    bool inside = false;

    /* A white run only counts once another ring pixel closes it, so the run
     * trailing off the right of the finder is simply never banked.  That
     * settles the row in one pass, with no need to locate its extents first. */
    for (int x = x0; x <= x1; x++) {
      quirc_pixel_t p = row[x];
      if (p == code) {
        ring++;
        if (inside)
          white += pending;
        inside = true;
        pending = 0;
      } else if (inside && p == QUIRC_PIXEL_WHITE) {
        pending++;
      }
    }
  }

  q->dilation_ring += ring;
  q->dilation_white += white;
}
#endif /* K_QUIRC_ADAPTIVE_THRESHOLD */

static void record_capstone(struct k_quirc *q, int ring, int stone) {
  struct quirc_region *stone_reg = &q->regions[stone];
  struct quirc_region *ring_reg = &q->regions[ring];
  struct quirc_capstone *capstone;
  int cs_index;

  if (q->num_capstones >= QUIRC_MAX_CAPSTONES)
    return;

  cs_index = q->num_capstones;
  capstone = &q->capstones[q->num_capstones++];

  memset(capstone, 0, sizeof(*capstone));

  capstone->qr_grid = -1;
  capstone->ring = ring;
  capstone->stone = stone;
  stone_reg->capstone = cs_index;
  ring_reg->capstone = cs_index;

  find_region_corners(q, ring, &stone_reg->seed, capstone->corners);
  if (!perspective_setup(capstone->c, capstone->corners, 7.0f, 7.0f)) {
    stone_reg->capstone = -1;
    ring_reg->capstone = -1;
    q->num_capstones = cs_index;
    return;
  }
  perspective_map(capstone->c, 3.5f, 3.5f, &capstone->center);

#ifdef K_QUIRC_ADAPTIVE_THRESHOLD
  accumulate_dilation(q, ring, capstone->corners);
#endif
}

/* Length of the dark (or light) run from (x, y) along (dx, dy), excluding
 * (x, y) itself */
static int line_run(const struct k_quirc *q, int x, int y, int dx, int dy,
                    bool dark) {
  const quirc_pixel_t *p = q->pixels + y * q->w + x;
  int stride = dy * q->w + dx;
  int room_x = dx > 0 ? q->w - 1 - x : dx < 0 ? x : INT_MAX;
  int room_y = dy > 0 ? q->h - 1 - y : dy < 0 ? y : INT_MAX;
  int room = room_x < room_y ? room_x : room_y;
  int n = 0;

  while (n < room && (p[stride] != QUIRC_PIXEL_WHITE) == dark) {
    p += stride;
    n++;
  }
  return n;
}

/* Does the line through (x, y) along (dx, dy) read a finder's 1:1:3:1:1, with
 * (x, y) in the stone?  `mid` gets the stone's middle, in steps from (x, y).
 *
 * Dense data matches a row by chance hundreds of times a frame; each match
 * cost flood fills and region labels, until none were left for the finders
 * further down.  But a finder is concentric squares, so every line through
 * its centre shows the same ratio, and data rarely does that four times.  The
 * runs are judged against their span of seven modules, which a fattened
 * binarization leaves alone, and loosely: a finder two pixels to the module
 * quantizes badly. */
static bool finder_cross_section(const struct k_quirc *q, int x, int y, int dx,
                                 int dy, int *mid) {
  int run[5]; /* ring, gap, stone, gap, ring */
  int back = line_run(q, x, y, -dx, -dy, true);
  int fwd = line_run(q, x, y, dx, dy, true);
  int bx = x - back * dx, by = y - back * dy;
  int fx = x + fwd * dx, fy = y + fwd * dy;

  run[2] = back + fwd + 1;
  run[1] = line_run(q, bx, by, -dx, -dy, false);
  run[3] = line_run(q, fx, fy, dx, dy, false);
  run[0] = line_run(q, bx - run[1] * dx, by - run[1] * dy, -dx, -dy, true);
  run[4] = line_run(q, fx + run[3] * dx, fy + run[3] * dy, dx, dy, true);
  *mid = (fwd - back) / 2;

  /* A gap or ring of 1/4 to 2 modules, a stone of 2 to 4.5 */
  int span = run[0] + run[1] + run[2] + run[3] + run[4];
  for (int i = 0; i < 5; i++) {
    if (i == 2 ? (14 * run[i] < 4 * span || 14 * run[i] > 9 * span)
               : (28 * run[i] < span || 28 * run[i] > 8 * span))
      return false;
  }
  return true;
}

/* The column through a candidate's stone finds the finder's middle row, that
 * row its centre, and the diagonals through the centre confirm it. */
static bool finder_cross_check(const struct k_quirc *q, int x, int y) {
  int mid;

  if (!finder_cross_section(q, x, y, 0, 1, &mid))
    return false;
  y += mid;
  if (!finder_cross_section(q, x, y, 1, 0, &mid))
    return false;
  x += mid;
  return finder_cross_section(q, x, y, 1, 1, &mid) &&
         finder_cross_section(q, x, y, 1, -1, &mid);
}

static void test_capstone(struct k_quirc *q, int x, int y, int *pb) {
  int ring_right_x = x - pb[4];
  int ring_left_x = x - pb[4] - pb[3] - pb[2] - pb[1] - pb[0];
  int stone_x = x - pb[4] - pb[3] - pb[2];

  /* Every row through a finder is a candidate; all but the first find it
   * already recorded. */
  int seen = q->pixels[y * q->w + stone_x];
  if (seen >= QUIRC_PIXEL_REGION && q->regions[seen].capstone >= 0)
    return;

  if (!finder_cross_check(q, stone_x + pb[2] / 2, y))
    return;

  int ring_right = region_code(q, ring_right_x, y);
  int ring_left = region_code(q, ring_left_x, y);

  if (ring_left < 0 || ring_right < 0)
    return;

  if (ring_left != ring_right)
    return;

  int stone = region_code(q, stone_x, y);
  if (stone < 0)
    return;

  if (ring_left == stone)
    return;

  struct quirc_region *stone_reg = &q->regions[stone];
  struct quirc_region *ring_reg = &q->regions[ring_left];

  if (stone_reg->capstone >= 0 || ring_reg->capstone >= 0)
    return;

  if (ring_reg->count <= 0)
    return;

  int ratio = stone_reg->count * 100 / ring_reg->count;
  if (ratio < 5 || ratio > 80)
    return;

  record_capstone(q, ring_left, stone);
}

static void finder_scan(struct k_quirc *q, int y) {
  quirc_pixel_t *row = q->pixels + y * q->w;
  int w = q->w;
  int run_count = 0;
  int pb[5];

  memset(pb, 0, sizeof(pb));

  quirc_pixel_t color = row[0];
  int start = 0;

  for (;;) {
    /* Word-at-a-time scan to the end of the current run */
    int x = row_run_end(row, start + 1, w, color);
    if (x >= w)
      break; /* last run of the row never completes */

    pb[0] = pb[1];
    pb[1] = pb[2];
    pb[2] = pb[3];
    pb[3] = pb[4];
    pb[4] = x - start;
    run_count++;

    quirc_pixel_t next_color = row[x];

    if (!next_color && run_count >= 5) {
      int avg = (pb[0] + pb[1] + pb[3] + pb[4]) >> 2;
      if (avg == 0)
        avg = 1;
      int err = (avg * 3) >> 2;

      /* Check 1:1:3:1:1 finder pattern ratio */
      int lo = avg - err;
      int hi = avg + err;
      int lo3 = avg * 3 - err;
      int hi3 = avg * 3 + err;

      if (pb[0] >= lo && pb[0] <= hi && pb[1] >= lo && pb[1] <= hi &&
          pb[2] >= lo3 && pb[2] <= hi3 && pb[3] >= lo && pb[3] <= hi &&
          pb[4] >= lo && pb[4] <= hi) {
        test_capstone(q, x, y, pb);
      }
    }

    color = next_color;
    start = x;
  }
}

/* Cells of the 5x5 alignment pattern that disagree with the image around a
 * centre, 16.16, a module being (ux, uy) along u and (vx, vy) along v.  The
 * pattern is symmetric, so which way the axes run is moot. */
static int alignment_pattern_errors(const struct k_quirc *q, int px, int py,
                                    int ux, int uy, int vx, int vy) {
  int errors = 0;

  for (int j = -2; j <= 2; j++) {
    for (int i = -2; i <= 2; i++) {
      int x = (px + i * ux + j * vx) >> 16;
      int y = (py + i * uy + j * vy) >> 16;
      int ring = abs(i) > abs(j) ? abs(i) : abs(j);
      bool dark = x >= 0 && y >= 0 && x < q->w && y < q->h &&
                  q->pixels[y * q->w + x] != QUIRC_PIXEL_WHITE;
      errors += dark != (ring != 1);
    }
  }
  return errors;
}

/* Half a module along a finder edge's direction at the corner without a
 * finder, 16.16.  Each finder measures the pitch where it stands, seven
 * modules to `edge`; across a plane in perspective it is the pitch's
 * reciprocal that varies linearly, so the fourth corner's is the two adjacent
 * finders' less the opposite one's. */
static bool far_corner_half_module(const struct k_quirc *q,
                                   const struct quirc_grid *qr, int edge,
                                   int *hx, int *hy) {
  float inv_len = 0.0f;
  float dx = 0.0f;
  float dy = 0.0f;

  for (int i = 0; i < 3; i++) {
    const struct quirc_point *c = q->capstones[qr->caps[i]].corners;
    float ex = (float)(c[edge].x - c[0].x);
    float ey = (float)(c[edge].y - c[0].y);
    float len = sqrtf(ex * ex + ey * ey);
    float sign = (i == 1) ? -1.0f : 1.0f;

    if (len < 1.0f)
      return false;
    inv_len += sign / len;
    dx += sign * ex / len;
    dy += sign * ey / len;
  }

  /* Near-parallel finder edges extrapolate to anything at all; no symbol's
   * seven modules outgrow the image */
  float norm = sqrtf(dx * dx + dy * dy);
  int dim = q->w > q->h ? q->w : q->h;
  if (inv_len * (float)dim < 1.0f || norm < 0.5f)
    return false;

  float k = 65536.0f / (14.0f * inv_len * norm);
  *hx = (int)(dx * k);
  *hy = (int)(dy * k);
  return true;
}

/* Of 25 cells.  Data mistaken for the pattern scores around 12. */
#define ALIGNMENT_PATTERN_MAX_ERRORS 3

/*
 * Locate the alignment pattern nearest the corner without a finder, the
 * fourth point the perspective hangs from, by its rings.
 *
 * Its centre is one dark module, and so is much of a dense code's data, while
 * the estimate to start from extends the finders' edges across the whole
 * symbol and lands a module or two out on a large one: the first region of a
 * module's size found near it is data about as often as not, and an anchor
 * that far out costs the read.  So slide the pattern over the neighbourhood in
 * half modules, and take the best match nearest the estimate.
 */
static void find_alignment_pattern(struct k_quirc *q, int index) {
  struct quirc_grid *qr = &q->grids[index];
  int ux, uy, vx, vy;

  if (qr->align.x < 0 || qr->align.y < 0 || qr->align.x >= q->w ||
      qr->align.y >= q->h)
    return;

  if (!far_corner_half_module(q, qr, 1, &ux, &uy) ||
      !far_corner_half_module(q, qr, 3, &vx, &vy))
    return;

  int x0 = qr->align.x * 65536 + 32768;
  int y0 = qr->align.y * 65536 + 32768;
  int reach = qr->grid_size / 10;
  int best = ALIGNMENT_PATTERN_MAX_ERRORS + 1;
  int best_i = 0, best_j = 0, best_d = 0;

  if (reach < 6)
    reach = 6;
  for (int j = -reach; j <= reach; j++) {
    for (int i = -reach; i <= reach; i++) {
      int errors = alignment_pattern_errors(q, x0 + i * ux + j * vx,
                                            y0 + i * uy + j * vy, 2 * ux,
                                            2 * uy, 2 * vx, 2 * vy);
      int d = i * i + j * j;
      if (errors < best || (errors == best && d < best_d)) {
        best = errors;
        best_i = i;
        best_j = j;
        best_d = d;
      }
    }
  }
  if (best > ALIGNMENT_PATTERN_MAX_ERRORS)
    return;

  /* The pattern matches over a small plateau around its centre, which is
   * the mean of the positions next to the winner that do as well */
  int n = 0, sum_i = 0, sum_j = 0;
  for (int j = best_j - 1; j <= best_j + 1; j++) {
    for (int i = best_i - 1; i <= best_i + 1; i++) {
      if (alignment_pattern_errors(q, x0 + i * ux + j * vx,
                                   y0 + i * uy + j * vy, 2 * ux, 2 * uy, 2 * vx,
                                   2 * vy) != best)
        continue;
      sum_i += i;
      sum_j += j;
      n++;
    }
  }
  qr->align.x = (x0 + (sum_i * ux + sum_j * vx) / n) >> 16;
  qr->align.y = (y0 + (sum_i * uy + sum_j * vy) / n) >> 16;

  /* The centre module's own centre of mass is finer still, where it stands
   * alone as it should */
  int module_area = abs((ux >> 7) * (vy >> 7) - (uy >> 7) * (vx >> 7)) >> 16;
  int code = region_code(q, qr->align.x, qr->align.y);
  if (code >= 0 && q->regions[code].count <= 3 * module_area)
    qr->align_region = code;
}

/*
 * Grid fitness: how well a code's known structure matches the image under a
 * candidate perspective, every cell sampled at its centre and at four corners
 * a fifth of a module out.
 *
 * Fitting evaluates a few hundred thousand cells and floating point is the
 * whole cost, so cell centres are walked along lines, where the map's
 * numerators and denominator advance by additions, and the samples around a
 * centre are integer steps along the map's Jacobian, taken once per pattern.
 */

/* Image-space steps, 16.16, for a fifth of a module along u and v, and how far
 * from a cell's centre its samples reach, in whole pixels. */
struct sample_steps {
  int xu, xv, yu, yv;
  int reach_x, reach_y;
};

static bool sample_steps_at(const float *c, float u, float v,
                            struct sample_steps *s) {
  float inv = 1.0f / (c[6] * u + c[7] * v + 1.0f);
  float px = (c[0] * u + c[1] * v + c[2]) * inv;
  float py = (c[3] * u + c[4] * v + c[5]) * inv;
  float k = inv * (0.2f * 65536.0f);
  float xu = (c[0] - c[6] * px) * k;
  float xv = (c[1] - c[7] * px) * k;
  float yu = (c[3] - c[6] * py) * k;
  float yv = (c[4] - c[7] * py) * k;

  /* A degenerate fit has no usable steps (NaN fails the test too) */
  if (!(fabsf(xu) + fabsf(xv) + fabsf(yu) + fabsf(yv) < 1e8f))
    return false;

  s->xu = (int)xu;
  s->xv = (int)xv;
  s->yu = (int)yu;
  s->yv = (int)yv;
  s->reach_x = ((abs(s->xu) + abs(s->xv)) >> 16) + 1;
  s->reach_y = ((abs(s->yu) + abs(s->yv)) >> 16) + 1;
  return true;
}

/* The perspective map at a cell centre, as numerators and denominator */
struct cell_cursor {
  const float *c;
  float nx, ny, den;
};

static void cursor_set(struct cell_cursor *k, const float *c, int x, int y) {
  float u = x + 0.5f;
  float v = y + 0.5f;
  k->c = c;
  k->nx = c[0] * u + c[1] * v + c[2];
  k->ny = c[3] * u + c[4] * v + c[5];
  k->den = c[6] * u + c[7] * v + 1.0f;
}

enum { DIR_U, DIR_V, DIR_BACK_U, DIR_BACK_V };

ALWAYS_INLINE int dark_at(const quirc_pixel_t *pixels, unsigned w, int fx,
                          int fy) {
  return pixels[(unsigned)(fy >> 16) * w + (unsigned)(fx >> 16)] != 0;
}

/* Score n cells from the cursor along dir, leaving it on the cell after the
 * last: +1 per dark sample, -1 per light; reversed on even cells if
 * `alternate`. */
HOT_FUNC
static int fitness_line(const struct k_quirc *q, const struct sample_steps *s,
                        struct cell_cursor *k, int dir, int n, bool alternate) {
  const quirc_pixel_t *pixels = q->pixels;
  const unsigned w = (unsigned)q->w;
  const unsigned h = (unsigned)q->h;
  const int xu = s->xu, xv = s->xv, yu = s->yu, yv = s->yv;
  const int reach_x = s->reach_x, reach_y = s->reach_y;
  const float *c = k->c + (dir & 1);
  const bool back = dir & 2;
  const float nx_step = back ? -c[0] : c[0];
  const float ny_step = back ? -c[3] : c[3];
  const float den_step = back ? -c[6] : c[6];
  float nx = k->nx, ny = k->ny, den = k->den;
  int score = 0;

  for (int i = 0; i < n; i++) {
    float inv = 65536.0f / den;
    float px = nx * inv;
    float py = ny * inv;

    /* Cells far outside the image score nothing, and would overflow below */
    if (fabsf(px) + fabsf(py) < 8192.0f * 65536.0f) {
      int fx = (int)px + 32768 - xu - xv; /* first sample; 32768 rounds */
      int fy = (int)py + 32768 - yu - yv;
      unsigned cx = (unsigned)(((int)px >> 16) - reach_x);
      unsigned cy = (unsigned)(((int)py >> 16) - reach_y);
      int cell = 0;

      if (LIKELY(cx < w - 2 * reach_x - 1 && cy < h - 2 * reach_y - 1 &&
                 w > 2u * reach_x + 1 && h > 2u * reach_y + 1)) {
        cell = dark_at(pixels, w, fx, fy) +
               dark_at(pixels, w, fx + 2 * xu, fy + 2 * yu) +
               dark_at(pixels, w, fx + xu + xv, fy + yu + yv) +
               dark_at(pixels, w, fx + 2 * xv, fy + 2 * yv) +
               dark_at(pixels, w, fx + 2 * (xu + xv), fy + 2 * (yu + yv));
        cell = 2 * cell - 5;
      } else {
        for (int j = 0; j < 3; j++) {
          for (int m = j & 1; m < 3; m += 2) {
            unsigned sx = (unsigned)((fx + m * xu) >> 16);
            unsigned sy = (unsigned)((fy + m * yu) >> 16);
            if (sx < w && sy < h)
              cell += pixels[sy * w + sx] ? 1 : -1;
          }
          fx += xv;
          fy += yv;
        }
      }
      score += (alternate && !(i & 1)) ? -cell : cell;
    }

    nx += nx_step;
    ny += ny_step;
    den += den_step;
  }

  k->nx = nx;
  k->ny = ny;
  k->den = den;
  return score;
}

/* Concentric square rings around (cx, cy), dark or light as `ring_sign` gives
 * them from the centre out.  Each ring is a closed path from its top-left
 * cell, and the next one in starts a cell down the diagonal. */
static int fitness_rings(const struct k_quirc *q, const float *c, int cx,
                         int cy, const int8_t *ring_sign, int rings) {
  struct sample_steps s;
  struct cell_cursor k;
  int score = 0;

  if (!sample_steps_at(c, cx + 0.5f, cy + 0.5f, &s))
    return 0;
  cursor_set(&k, c, cx - (rings - 1), cy - (rings - 1));

  for (int r = rings - 1; r > 0; r--) {
    int ring = 0;
    for (int dir = DIR_U; dir <= DIR_BACK_V; dir++)
      ring += fitness_line(q, &s, &k, dir, 2 * r, false);
    score += ring_sign[r] * ring;

    k.nx += c[0] + c[1];
    k.ny += c[3] + c[4];
    k.den += c[6] + c[7];
  }

  return score + ring_sign[0] * fitness_line(q, &s, &k, DIR_U, 1, false);
}

static int fitness_apat(const struct k_quirc *q, const float *c, int cx,
                        int cy) {
  static const int8_t sign[] = {1, -1, 1};
  return fitness_rings(q, c, cx, cy, sign, 3);
}

static int fitness_capstone(const struct k_quirc *q, const float *c, int x,
                            int y) {
  static const int8_t sign[] = {1, 1, -1, 1};
  return fitness_rings(q, c, x + 3, y + 3, sign, 4);
}

/* A timing pattern: n alternating cells from (x, y) along dir, the sampling
 * steps refreshed every few cells. */
static int fitness_timing(const struct k_quirc *q, const float *c, int x, int y,
                          int dir, int n) {
  const int segment = 8; /* even: every segment starts on a light cell */
  struct cell_cursor k;
  int score = 0;

  cursor_set(&k, c, x, y);
  for (int i = 0; i < n; i += segment) {
    int len = (n - i < segment) ? n - i : segment;
    float mid = i + 0.5f * len;
    struct sample_steps s;
    if (!sample_steps_at(c, x + 0.5f + (dir == DIR_U ? mid : 0.0f),
                         y + 0.5f + (dir == DIR_V ? mid : 0.0f), &s))
      return score;
    score += fitness_line(q, &s, &k, dir, len, true);
  }
  return score;
}

static int fitness_all(const struct k_quirc *q, int index) {
  const struct quirc_grid *qr = &q->grids[index];
  const float *c = qr->c;
  int gs = qr->grid_size;
  int version = (gs - 17) / 4;
  int ap_count;

  int score = fitness_timing(q, c, 7, 6, DIR_U, gs - 14) +
              fitness_timing(q, c, 6, 7, DIR_V, gs - 14) +
              fitness_capstone(q, c, 0, 0) + fitness_capstone(q, c, gs - 7, 0) +
              fitness_capstone(q, c, 0, gs - 7);

  if (version < 0 || version > QUIRC_MAX_VERSION)
    return score;

  const struct quirc_version_info *info = &quirc_version_db[version];
  ap_count = 0;
  while ((ap_count < QUIRC_MAX_ALIGNMENT) && info->apat[ap_count])
    ap_count++;

  for (int i = 1; i + 1 < ap_count; i++) {
    score += fitness_apat(q, c, 6, info->apat[i]);
    score += fitness_apat(q, c, info->apat[i], 6);
  }

  /* Up to 36 inner patterns over-determine an eight-parameter map; every
   * other one will do, counted back from the far corner, whose pattern is the
   * only anchor there. */
  int stride = (ap_count >= 4) ? 2 : 1;
  for (int i = ap_count - 1; i >= 1; i -= stride)
    for (int j = ap_count - 1; j >= 1; j -= stride)
      score += fitness_apat(q, c, info->apat[i], info->apat[j]);

  return score;
}

#ifdef K_QUIRC_ADAPTIVE_THRESHOLD
static int clamp_threshold_offset(int offset) {
  if (offset > K_QUIRC_THRESHOLD_OFFSET_MAX)
    return K_QUIRC_THRESHOLD_OFFSET_MAX;
  if (offset < -K_QUIRC_THRESHOLD_OFFSET_MAX)
    return -K_QUIRC_THRESHOLD_OFFSET_MAX;
  return offset;
}

/*
 * Close the loop on the finder areas gathered this frame.  The correction is
 * proportional to the measured error, so the operating point is reached in a
 * frame or two rather than one gray level at a time -- on an animated QR the
 * frames spent travelling are payload parts missed.
 */
static void update_threshold_offset(struct k_quirc *q) {
  uint32_t ring = q->dilation_ring;
  uint32_t white = q->dilation_white;

  float den = 4.0f * (float)ring + 6.0f * (float)white;
  if (den <= 0.0f) {
    /* Nothing to measure: leak one level towards the default.  Finder
     * detection itself fails at the extremes of the range, so an offset that
     * suited one scene can leave the loop unable to see the next; leaking
     * walks it back until measurement resumes.  At one level per frame a
     * subject that briefly left the frame loses almost nothing. */
    if (q->threshold_offset > default_threshold_offset)
      q->threshold_offset--;
    else if (q->threshold_offset < default_threshold_offset)
      q->threshold_offset++;
    return;
  }

  /* Steer the measured dilation to zero: the finders then measure the size
   * they are specified to be.  There is no fitted set point here. */
  float correction =
      -K_QUIRC_DILATION_GAIN * (2.0f * (float)ring - 3.0f * (float)white) / den;

  int step = (int)(correction + (correction >= 0.0f ? 0.5f : -0.5f));
  if (step > -K_QUIRC_THRESHOLD_STEP_MIN && step < K_QUIRC_THRESHOLD_STEP_MIN)
    return; /* inside the deadband: already at the operating point */
  if (step > K_QUIRC_THRESHOLD_STEP_MAX)
    step = K_QUIRC_THRESHOLD_STEP_MAX;
  else if (step < -K_QUIRC_THRESHOLD_STEP_MAX)
    step = -K_QUIRC_THRESHOLD_STEP_MAX;

  q->threshold_offset = clamp_threshold_offset(q->threshold_offset + step);
}

int k_quirc_get_threshold_offset(void) { return default_threshold_offset; }
void k_quirc_set_threshold_offset(int offset) {
  default_threshold_offset = clamp_threshold_offset(offset);
}
int k_quirc_get_threshold_offset_for(const struct k_quirc *q) {
  return q ? q->threshold_offset : default_threshold_offset;
}
void k_quirc_set_threshold_offset_for(struct k_quirc *q, int offset) {
  if (q)
    q->threshold_offset = clamp_threshold_offset(offset);
}
#else
int k_quirc_get_threshold_offset(void) { return 0; }
void k_quirc_set_threshold_offset(int offset) { (void)offset; }
int k_quirc_get_threshold_offset_for(const struct k_quirc *q) {
  (void)q;
  return 0;
}
void k_quirc_set_threshold_offset_for(struct k_quirc *q, int offset) {
  (void)q;
  (void)offset;
}
#endif

#define JIGGLE_PASSES 2

static void jiggle_perspective(struct k_quirc *q, int index) {
  struct quirc_grid *qr = &q->grids[index];
  int best = fitness_all(q, index);
  float adjustments[8];

  float step_factor = 0.42f / (float)qr->grid_size;
  for (int i = 0; i < 8; i++)
    adjustments[i] = qr->c[i] * step_factor;

  for (int pass = 0; pass < JIGGLE_PASSES; pass++) {
    for (int i = 0; i < 16; i++) {
      int j = i >> 1;
      float old = qr->c[j];
      float step = adjustments[j];
      float new_val = (i & 1) ? old + step : old - step;

      if (new_val == old)
        continue; /* a zero coefficient: the same map again */
      qr->c[j] = new_val;
      int test = fitness_all(q, index);

      if (test > best) {
        best = test;
        i |= 1; /* stepping back up from here is the value just beaten */
      } else {
        qr->c[j] = old;
      }
    }

    for (int i = 0; i < 8; i++)
      adjustments[i] *= 0.5f;
  }
}

static int setup_qr_perspective(struct k_quirc *q, int index) {
  struct quirc_grid *qr = &q->grids[index];
  float gs = (float)qr->grid_size;

  struct quirc_point *c0 = &q->capstones[qr->caps[0]].center;
  struct quirc_point *c1 = &q->capstones[qr->caps[1]].center;
  struct quirc_point *c2 = &q->capstones[qr->caps[2]].center;

  float img[4][2] = {{(float)c1->x, (float)c1->y},
                     {(float)c2->x, (float)c2->y},
                     {(float)qr->align.x, (float)qr->align.y},
                     {(float)c0->x, (float)c0->y}};

  float mod[4][2] = {{3.5f, 3.5f},
                     {gs - 3.5f, 3.5f},
                     {gs - 6.5f, gs - 6.5f},
                     {3.5f, gs - 3.5f}};

  if (qr->grid_size == 21) {
    mod[2][0] = gs - 7.0f;
    mod[2][1] = gs - 7.0f;
  }

  if (!perspective_setup_direct(qr->c, img, mod))
    return 0;

  jiggle_perspective(q, index);
  return 1;
}

static float length(struct quirc_point a, struct quirc_point b) {
  float dx = (float)(abs(a.x - b.x) + 1);
  float dy = (float)(abs(a.y - b.y) + 1);
  return sqrtf(dx * dx + dy * dy);
}

/*
 * From version 7 up a code states its version: six bits and twelve of BCH
 * parity, in a 3x6 block left of the top-right finder and again above the
 * bottom-left one.  Both are read through the finder's own perspective, where
 * the block is cells -4..-2 by 0..5.
 */
static uint32_t version_codeword(int version) {
  uint32_t rem = (uint32_t)version << 12;
  for (int i = 17; i >= 12; i--)
    if (rem & (1u << i))
      rem ^= 0x1f25u << (i - 12);
  return ((uint32_t)version << 12) | rem;
}

static uint32_t read_version_block(const struct k_quirc *q,
                                   const struct quirc_capstone *cap,
                                   bool left_of) {
  struct sample_steps s;
  uint32_t word = 0;

  if (!sample_steps_at(cap->c, left_of ? -2.5f : 3.0f, left_of ? 3.0f : -2.5f,
                       &s))
    return 0;

  for (int bit = 0; bit < 18; bit++) {
    int across = bit % 3 - 4;
    int along = bit / 3;
    struct cell_cursor k;
    cursor_set(&k, cap->c, left_of ? across : along, left_of ? along : across);
    if (fitness_line(q, &s, &k, DIR_U, 1, false) > 0)
      word |= 1u << bit;
  }
  return word;
}

static int bit_errors(uint32_t word, int version) {
  int errors = 0;
  for (uint32_t x = word ^ version_codeword(version); x; x &= x - 1)
    errors++;
  return errors;
}

/* The stated version, if one within two of the estimate reads with the three
 * bit errors the parity corrects; else the estimate.  Below version 7 there
 * is only data where the block would be, and one random word in seventy
 * passes, so while the estimate leaves that open both copies must agree, or
 * one be exact. */
static int stated_version(const struct k_quirc *q, const struct quirc_grid *qr,
                          int estimate) {
  if (estimate < 5)
    return estimate;

  uint32_t top_right = read_version_block(q, &q->capstones[qr->caps[2]], true);
  uint32_t bottom_left =
      read_version_block(q, &q->capstones[qr->caps[0]], false);
  int best = estimate;
  int best_errors = 4;

  for (int v = estimate - 2; v <= estimate + 2; v++) {
    if (v < 7 || v > QUIRC_MAX_VERSION)
      continue;
    int e0 = bit_errors(top_right, v);
    int e1 = bit_errors(bottom_left, v);
    bool fits = (estimate >= 7) ? (e0 <= 3 || e1 <= 3)
                                : ((e0 <= 3 && e1 <= 3) || !e0 || !e1);
    int errors = (e0 < e1) ? e0 : e1;
    if (fits && errors < best_errors) {
      best_errors = errors;
      best = v;
    }
  }
  return best;
}

static void measure_grid_size(struct k_quirc *q, int index) {
  struct quirc_grid *qr = &q->grids[index];

  struct quirc_capstone *a = &(q->capstones[qr->caps[0]]);
  struct quirc_capstone *b = &(q->capstones[qr->caps[1]]);
  struct quirc_capstone *c = &(q->capstones[qr->caps[2]]);

  float ab = length(b->corners[0], a->corners[3]);
  float capstone_ab_size = (length(b->corners[0], b->corners[3]) +
                            length(a->corners[0], a->corners[3])) *
                           0.5f;
  float ver_grid = 7.0f * ab / capstone_ab_size;

  float bc = length(b->corners[0], c->corners[1]);
  float capstone_bc_size = (length(b->corners[0], b->corners[1]) +
                            length(c->corners[0], c->corners[1])) *
                           0.5f;
  float hor_grid = 7.0f * bc / capstone_bc_size;

  float grid_size_estimate = (ver_grid + hor_grid) * 0.5f;

  /* Finder size against finder spacing is good to a version or so: a pixel's
   * error in a corner is already a few percent. */
  int ver = (int)((grid_size_estimate - 15.0f) * 0.25f);
  ver = stated_version(q, qr, ver);
  if (ver > QUIRC_MAX_VERSION) {
    /* Reject rather than clamp. Clamping produced a QUIRC_MAX_VERSION grid for
     * a symbol that is physically larger, so sampling ran on the wrong lattice
     * and the code failed data ECC after a full Reed-Solomon pass - a silent
     * misread dressed up as a decode failure. Dropping the candidate here
     * reports nothing and skips that wasted work. Callers treat grid_size 0 as
     * "no grid" via the < 21 test in record_qr_grid(). */
    K_QUIRC_LOGD(TAG, "QR version %d exceeds supported maximum %d", ver,
                 QUIRC_MAX_VERSION);
    qr->grid_size = 0;
    return;
  }

  qr->grid_size = 4 * ver + 17;
}

static K_QUIRC_WARN_UNUSED_RESULT int
rotate_capstone(struct quirc_capstone *cap, const struct quirc_point *h0,
                const struct quirc_point *hd) {
  struct quirc_point copy[4];
  float c[QUIRC_PERSPECTIVE_PARAMS];
  int best = 0;
  int best_score = 0;

  for (int j = 0; j < 4; j++) {
    struct quirc_point *p = &cap->corners[j];
    int score = (p->x - h0->x) * -hd->y + (p->y - h0->y) * hd->x;

    if (!j || score < best_score) {
      best = j;
      best_score = score;
    }
  }

  for (int j = 0; j < 4; j++)
    memcpy(&copy[j], &cap->corners[(j + best) % 4], sizeof(copy[j]));
  if (!perspective_setup(c, copy, 7.0f, 7.0f))
    return 0;

  memcpy(cap->corners, copy, sizeof(cap->corners));
  memcpy(cap->c, c, sizeof(c));
  return 1;
}

static void record_qr_grid(struct k_quirc *q, int a, int b, int c) {
  struct quirc_point h0, hd;
  struct quirc_grid *qr;

  if (q->num_grids >= QUIRC_MAX_GRIDS)
    return;

  memcpy(&h0, &q->capstones[a].center, sizeof(h0));
  hd.x = q->capstones[c].center.x - q->capstones[a].center.x;
  hd.y = q->capstones[c].center.y - q->capstones[a].center.y;

  if ((q->capstones[b].center.x - h0.x) * -hd.y +
          (q->capstones[b].center.y - h0.y) * hd.x >
      0) {
    int swap = a;
    a = c;
    c = swap;
    hd.x = -hd.x;
    hd.y = -hd.y;
  }

  qr = &q->grids[q->num_grids];
  memset(qr, 0, sizeof(*qr));
  qr->caps[0] = a;
  qr->caps[1] = b;
  qr->caps[2] = c;
  qr->align_region = -1;

  for (int i = 0; i < 3; i++) {
    struct quirc_capstone *cap = &q->capstones[qr->caps[i]];
    if (!rotate_capstone(cap, &h0, &hd))
      return;
  }

  measure_grid_size(q, q->num_grids);

  if (qr->grid_size < 21)
    return;

  if (qr->grid_size > 177)
    return;

  if (!line_intersect(&q->capstones[a].corners[0], &q->capstones[a].corners[1],
                      &q->capstones[c].corners[0], &q->capstones[c].corners[3],
                      &qr->align))
    return;

  if (qr->grid_size > 21) {
    find_alignment_pattern(q, q->num_grids);
    if (qr->align_region >= 0) {
      /* Use the centroid of the alignment region, not the seed point.
       * The seed is just the first pixel the spiral search landed on,
       * which can be at the edge of the region rather than its center.
       * This offset skews the perspective transform and causes decode
       * failures, especially for low-version QR codes captured at
       * high resolution where each module consists of many pixels. */
      struct quirc_region *areg = &q->regions[qr->align_region];
      int code = qr->align_region;
      long sum_x = 0, sum_y = 0;
      int n = 0;
      for (int sy = areg->y0; sy <= areg->y1; sy++) {
        for (int sx = areg->x0; sx <= areg->x1; sx++) {
          if (q->pixels[sy * q->w + sx] == code) {
            sum_x += sx;
            sum_y += sy;
            n++;
          }
        }
      }
      if (n > 0) {
        qr->align.x = (int)(sum_x / n);
        qr->align.y = (int)(sum_y / n);
      } else {
        memcpy(&qr->align, &areg->seed, sizeof(qr->align));
      }
    }
  }

  if (!setup_qr_perspective(q, q->num_grids))
    return;

  for (int i = 0; i < 3; i++)
    q->capstones[qr->caps[i]].qr_grid = q->num_grids;

  q->num_grids++;
}

/*
 * Geometric grouping: detect right-angle triangles formed by capstone centers.
 *
 * In a valid QR code the three finder patterns always form a right-angle
 * triangle.  The right-angle vertex is the "top-left" capstone in QR
 * terminology.  We detect this directly from image-space distances using the
 * Pythagorean theorem, which is far more robust than perspective_unmap when
 * the QR code is viewed at a steep angle or is very dense.
 */
ALWAYS_INLINE float dist2(const struct quirc_point *a,
                          const struct quirc_point *b) {
  float dx = (float)(a->x - b->x);
  float dy = (float)(a->y - b->y);
  return dx * dx + dy * dy;
}

/* Maximum candidate triplets to evaluate before selecting the best */
#define MAX_CANDIDATES 16

struct triplet_candidate {
  int leg1, vertex, leg2;
  float score; /* higher = better (leg_ratio * (1 - pyth_error)) */
};

static void geometric_grouping(struct k_quirc *q) {
  int n = q->num_capstones;
  struct triplet_candidate candidates[MAX_CANDIDATES];
  int ncand = 0;

  /* Collect all valid right-angle triplets */
  for (int i = 0; i < n - 2; i++) {
    for (int j = i + 1; j < n - 1; j++) {
      for (int k = j + 1; k < n; k++) {

        float d2_ij = dist2(&q->capstones[i].center, &q->capstones[j].center);
        float d2_ik = dist2(&q->capstones[i].center, &q->capstones[k].center);
        float d2_jk = dist2(&q->capstones[j].center, &q->capstones[k].center);

        /* Find hypotenuse (longest side).  The vertex opposite it has the
         * right angle and corresponds to the top-left finder pattern. */
        float d2_hyp, d2_a, d2_b;
        int vertex, leg1, leg2;

        if (d2_jk >= d2_ij && d2_jk >= d2_ik) {
          d2_hyp = d2_jk;
          d2_a = d2_ij;
          d2_b = d2_ik;
          vertex = i;
          leg1 = j;
          leg2 = k;
        } else if (d2_ik >= d2_ij && d2_ik >= d2_jk) {
          d2_hyp = d2_ik;
          d2_a = d2_ij;
          d2_b = d2_jk;
          vertex = j;
          leg1 = i;
          leg2 = k;
        } else {
          d2_hyp = d2_ij;
          d2_a = d2_ik;
          d2_b = d2_jk;
          vertex = k;
          leg1 = i;
          leg2 = j;
        }

        /* Pythagorean check: a² + b² ≈ c²  (right-angle test) */
        float expected = d2_a + d2_b;
        if (expected < 100.0f)
          continue;

        float pyth_error = fabsf(1.0f - d2_hyp / expected);
        if (pyth_error > 0.25f)
          continue;

        /* Leg ratio: reject if legs are wildly different (squared) */
        float leg_ratio = (d2_a < d2_b) ? d2_a / d2_b : d2_b / d2_a;
        if (leg_ratio < 0.1f)
          continue;

        if (ncand < MAX_CANDIDATES) {
          candidates[ncand].leg1 = leg1;
          candidates[ncand].vertex = vertex;
          candidates[ncand].leg2 = leg2;
          /* Score: prefer square triangles with perfect right angles */
          candidates[ncand].score = leg_ratio * (1.0f - pyth_error);
          ncand++;
        }
      }
    }
  }

  /* Sort candidates by score descending (simple insertion sort) */
  for (int i = 1; i < ncand; i++) {
    struct triplet_candidate tmp = candidates[i];
    int j = i - 1;
    while (j >= 0 && candidates[j].score < tmp.score) {
      candidates[j + 1] = candidates[j];
      j--;
    }
    candidates[j + 1] = tmp;
  }

  /* Record grids in score order, skipping capstones already used */
  for (int i = 0; i < ncand; i++) {
    if (q->num_grids >= QUIRC_MAX_GRIDS)
      return;
    int l1 = candidates[i].leg1;
    int v = candidates[i].vertex;
    int l2 = candidates[i].leg2;
    if (q->capstones[l1].qr_grid >= 0 || q->capstones[v].qr_grid >= 0 ||
        q->capstones[l2].qr_grid >= 0)
      continue;
    record_qr_grid(q, l1, v, l2);
  }
}

static void pixels_setup(struct k_quirc *q) {
  if (sizeof(*q->image) == sizeof(*q->pixels)) {
    q->pixels = (quirc_pixel_t *)q->image;
  } else {
    int total = q->w * q->h;
    for (int i = 0; i < total; i++)
      q->pixels[i] = q->image[i];
  }
}

/*
 * Public identification function
 */
void k_quirc_identify(struct k_quirc *q, bool find_inverted) {
  if (!q)
    return;

#ifdef K_QUIRC_ADAPTIVE_THRESHOLD
  q->dilation_ring = 0;
  q->dilation_white = 0;
#endif
  pixels_setup(q);
  threshold(q, false);

  /* A finder shows on several rows, so every other row finds it at half the
   * cost; the rows between are for when that came up short. */
  for (int i = 0; i < q->h; i += 2)
    finder_scan(q, i);
  if (q->num_capstones < 3) {
    for (int i = 1; i < q->h; i += 2)
      finder_scan(q, i);
  }

#ifdef K_QUIRC_ADAPTIVE_THRESHOLD
  /* Every capstone found above has contributed its finder areas.  Correct the
   * offset now, before grouping: the measurement does not depend on a grid
   * forming, so a frame that finds finders but fails to group still steers the
   * next one.  Consuming the sums here is also what keeps the inverted retry
   * out of them -- it re-thresholds the same frame, and mixing two
   * binarizations into one measurement would be meaningless. */
  update_threshold_offset(q);
#endif

  geometric_grouping(q);

#ifdef K_QUIRC_INVERTED_RETRY
  if (q->num_grids == 0 && find_inverted) {
    K_QUIRC_YIELD();
    q->num_regions = QUIRC_PIXEL_REGION;
    q->num_capstones = 0;
    q->num_grids = 0;

    pixels_setup(q);
    threshold(q, true);

    for (int i = 0; i < q->h; i++)
      finder_scan(q, i);

    geometric_grouping(q);
  }
#else
  (void)find_inverted;
#endif
}
