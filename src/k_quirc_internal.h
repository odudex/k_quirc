/*
 * K-Quirc Internal Header
 * Internal types, macros, and shared structures
 */

#ifndef K_QUIRC_INTERNAL_H
#define K_QUIRC_INTERNAL_H

#include "k_quirc.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>

#if !defined(K_QUIRC_LOGW) || !defined(K_QUIRC_LOGE) ||                        \
    !defined(K_QUIRC_LOGI) || !defined(K_QUIRC_LOGD)
#include <esp_log.h>
#endif
#ifndef K_QUIRC_LOGW
#define K_QUIRC_LOGW(tag, fmt, ...) ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#endif
#ifndef K_QUIRC_LOGE
#define K_QUIRC_LOGE(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)
#endif
#ifndef K_QUIRC_LOGI
#define K_QUIRC_LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#endif
#ifndef K_QUIRC_LOGD
#define K_QUIRC_LOGD(tag, fmt, ...) ESP_LOGD(tag, fmt, ##__VA_ARGS__)
#endif
#ifndef K_QUIRC_YIELD
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#define K_QUIRC_YIELD() vTaskDelay(1)
#endif

static inline void *k_malloc_large(size_t size) {
  void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT |
                                         MALLOC_CAP_CACHE_ALIGNED);
  if (!ptr)
    ptr = heap_caps_malloc(size, MALLOC_CAP_8BIT);
  return ptr;
}

static inline void *k_malloc_fast(size_t size) {
  void *ptr = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!ptr)
    ptr = k_malloc_large(size);
  return ptr;
}

#define K_MALLOC(size) k_malloc_large(size)
#define K_MALLOC_FAST(size) k_malloc_fast(size)
#define K_MALLOC_CONTEXT(size) k_malloc_fast(size)
#define K_MALLOC_IMAGE(size) k_malloc_large(size)
#define K_MALLOC_SCRATCH(size) k_malloc_fast(size)
#define K_FREE(ptr) heap_caps_free(ptr)
#else
#define K_MALLOC(size) malloc(size)
#define K_MALLOC_FAST(size) malloc(size)
#define K_MALLOC_CONTEXT(size) malloc(size)
#define K_MALLOC_IMAGE(size) malloc(size)
#define K_MALLOC_SCRATCH(size) malloc(size)
#define K_FREE(ptr) free(ptr)

#if !defined(K_QUIRC_LOGW) || !defined(K_QUIRC_LOGE) ||                        \
    !defined(K_QUIRC_LOGI) || !defined(K_QUIRC_LOGD)
#include <stdio.h>
#endif
#ifndef K_QUIRC_LOGW
#define K_QUIRC_LOGW(tag, fmt, ...)                                            \
  fprintf(stderr, "W (%s) " fmt "\n", tag, ##__VA_ARGS__)
#endif
#ifndef K_QUIRC_LOGE
#define K_QUIRC_LOGE(tag, fmt, ...)                                            \
  fprintf(stderr, "E (%s) " fmt "\n", tag, ##__VA_ARGS__)
#endif
#ifndef K_QUIRC_LOGI
#define K_QUIRC_LOGI(tag, fmt, ...)                                            \
  fprintf(stderr, "I (%s) " fmt "\n", tag, ##__VA_ARGS__)
#endif
#ifndef K_QUIRC_LOGD
#define K_QUIRC_LOGD(tag, fmt, ...)                                            \
  fprintf(stderr, "D (%s) " fmt "\n", tag, ##__VA_ARGS__)
#endif
#ifndef K_QUIRC_YIELD
#define K_QUIRC_YIELD() ((void)0)
#endif
#endif

/* Compiler optimization hints */
#ifndef __GNUC__
#define __attribute__(x)
#endif

#define HOT_FUNC __attribute__((hot))
#define ALWAYS_INLINE __attribute__((always_inline)) static inline
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

/* Quirc internal definitions */
#define QUIRC_PIXEL_WHITE 0
#define QUIRC_PIXEL_BLACK 1
#define QUIRC_PIXEL_REGION 2

#ifndef QUIRC_MAX_REGIONS
#define QUIRC_MAX_REGIONS 254
#endif

#define QUIRC_MAX_CAPSTONES 32
#define QUIRC_MAX_GRIDS 8
#define QUIRC_PERSPECTIVE_PARAMS 8
#define QUIRC_MAX_VERSION 27
#define QUIRC_MAX_ALIGNMENT 7
/* Flood-fill span stack: entries are 8 bytes each (see xylf_t), so the
 * default costs 64 KB of fast RAM.  Override to trade memory for the
 * ability to fill extremely fragmented regions without overflow. */
#ifndef QUIRC_FLOOD_FILL_STACK
#define QUIRC_FLOOD_FILL_STACK 8192
#endif

/* Flood-fill stack entry (defined here so allocation can use its size) */
typedef struct {
  int16_t x, y, l, r;
} xylf_t;
#define K_QUIRC_MAX_IMAGE_DIM 1280
#define K_QUIRC_THRESHOLD_OFFSET_DEFAULT 10

/* Adaptive threshold control loop: measures frame N, corrects frame N+1.
 *
 * The offset accumulates while the measurement settles within the frame, so
 * this is an integrating controller on a static plant and the error decays as
 * e[n+1] = (1 - GAIN*Kp) e[n], for a plant gain Kp = d(dilation)/d(offset).
 * Kp spans 0.0005 to 0.0033 over real captures, so the loop gain is 0.09 to
 * 0.66 and the pole stays real and positive: the approach is monotone from
 * any starting offset.  Overshoot would need a plant 1.5x faster than any
 * measured and instability 3x, so GAIN is set well below the deadbeat value
 * of ~550 for margin rather than for the shortest settling time.
 *
 * OFFSET_MAX bounds the operating point; the +/-20 this used to allow was far
 * short of the +50 or more that defocused, overexposed captures need.
 * STEP_MAX rate-limits one frame's measurement.  STEP_MIN is a deadband:
 * thresholding is a step function of an integer gray level, so without one
 * the loop settles into a limit cycle rather than a value -- deterministically,
 * on a perfectly repeated frame.  Widening it from 3 to 10 removed 38 of 39
 * hunting runs on real captures and raised yield from 39% to 43%.
 */
#ifndef K_QUIRC_THRESHOLD_OFFSET_MAX
#define K_QUIRC_THRESHOLD_OFFSET_MAX 70
#endif
#ifndef K_QUIRC_THRESHOLD_STEP_MAX
#define K_QUIRC_THRESHOLD_STEP_MAX 30
#endif
#ifndef K_QUIRC_THRESHOLD_STEP_MIN
#define K_QUIRC_THRESHOLD_STEP_MIN 10
#endif
#ifndef K_QUIRC_DILATION_GAIN
#define K_QUIRC_DILATION_GAIN 200.0f
#endif

#if QUIRC_MAX_REGIONS < UINT8_MAX
typedef uint8_t quirc_pixel_t;
#elif QUIRC_MAX_REGIONS < UINT16_MAX
typedef uint16_t quirc_pixel_t;
#else
#error "QUIRC_MAX_REGIONS > 65534 is not supported"
#endif

/*
 * Internal structures
 */
struct quirc_point {
  int x;
  int y;
};

struct quirc_region {
  struct quirc_point seed;
  int count;
  int capstone;
};

struct quirc_capstone {
  int ring;
  int stone;
  struct quirc_point corners[4];
  struct quirc_point center;
  float c[QUIRC_PERSPECTIVE_PARAMS];
  int qr_grid;
};

struct quirc_grid {
  int caps[3];
  int align_region;
  struct quirc_point align;
  int grid_size;
  float c[QUIRC_PERSPECTIVE_PARAMS];
};

struct quirc_code {
  struct quirc_point corners[4];
  int size;
  uint8_t cell_bitmap[K_QUIRC_MAX_BITMAP];
};

struct quirc_data {
  int version;
  int ecc_level;
  int mask;
  int data_type;
  uint8_t payload[K_QUIRC_MAX_PAYLOAD];
  int payload_len;
  uint32_t eci;
};

struct datastream {
  uint8_t raw[K_QUIRC_MAX_PAYLOAD];
  int data_bits;
  int ptr;
  uint8_t data[K_QUIRC_MAX_PAYLOAD];
};

struct k_quirc {
  uint8_t *image;
  quirc_pixel_t *pixels;
  uint8_t *flood_fill_stack;
  bool owns_pixels;
  bool flood_fill_overflow;
#ifdef K_QUIRC_ADAPTIVE_THRESHOLD
  int threshold_offset;
  /* Finder-pattern areas summed over every capstone found this frame.  Kept
   * as raw sums rather than per-capstone estimates so the whole frame costs
   * one division, and so larger (more reliable) finders carry more weight. */
  uint32_t dilation_ring;
  uint32_t dilation_white;
#endif
  int w;
  int h;
  int num_regions;
  struct quirc_region regions[QUIRC_MAX_REGIONS];
  int num_capstones;
  struct quirc_capstone capstones[QUIRC_MAX_CAPSTONES];
  int num_grids;
  struct quirc_grid grids[QUIRC_MAX_GRIDS];
  struct quirc_code code_scratch;
  struct quirc_data data_scratch;
  struct datastream ds_scratch;
};

/*
 * Version info structure
 */
struct quirc_rs_params {
  uint8_t bs;
  uint8_t dw;
  uint8_t ns;
};

struct quirc_version_info {
  uint16_t data_bytes;
  uint8_t apat[QUIRC_MAX_ALIGNMENT];
  struct quirc_rs_params ecc[4];
};

/* Version database - defined in k_quirc_version.c */
extern const struct quirc_version_info quirc_version_db[QUIRC_MAX_VERSION + 1];

/*
 * Helper functions
 */
ALWAYS_INLINE int fast_roundf(float x) { return (int)(x + 0.5f); }

ALWAYS_INLINE void perspective_map(const float *c, float u, float v,
                                   struct quirc_point *ret) {
  float den = c[6] * u + c[7] * v + 1.0f;
  float inv_den = 1.0f / den;
  float x = (c[0] * u + c[1] * v + c[2]) * inv_den;
  float y = (c[3] * u + c[4] * v + c[5]) * inv_den;

  ret->x = fast_roundf(x);
  ret->y = fast_roundf(y);
}

/*
 * Identification module functions (k_quirc_identify.c)
 */
void k_quirc_identify(struct k_quirc *q, bool find_inverted);
int k_quirc_get_threshold_offset(void);
void k_quirc_set_threshold_offset(int offset);
int k_quirc_get_threshold_offset_for(const struct k_quirc *q);
void k_quirc_set_threshold_offset_for(struct k_quirc *q, int offset);

/*
 * Decode module functions (k_quirc_decode.c)
 */
void quirc_extract_internal(const struct k_quirc *q, int index,
                            struct quirc_code *code);
void quirc_extract_nudged(const struct k_quirc *q, int index,
                          struct quirc_code *code, float du, float dv);
k_quirc_error_t quirc_decode_internal(const struct quirc_code *code,
                                      struct quirc_data *data,
                                      struct datastream *ds);

#endif /* K_QUIRC_INTERNAL_H */
