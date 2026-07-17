/*
 * K-Quirc - Standalone QR Code Recognition Library
 * Adapted from OpenMV's quirc implementation for single-core ESP32 use
 *
 * Original Copyright (C) 2010-2012 Daniel Beer <dlbeer@gmail.com>
 * OpenMV modifications Copyright (c) 2013-2021 Ibrahim Abdelkader
 * <iabdalkader@openmv.io> OpenMV modifications Copyright (c) 2013-2021 Kwabena
 * W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for
 * details.
 */

#ifndef K_QUIRC_H
#define K_QUIRC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Limits on the maximum size of QR-codes and their content (Max Version 27). */
#define K_QUIRC_MAX_BITMAP 1954  /* ceil(125*125/8) for version 27 */
#define K_QUIRC_MAX_PAYLOAD 2560 /* v27 total codewords 1990 < 2560 */

/* QR-code ECC types. */
#define K_QUIRC_ECC_LEVEL_M 0
#define K_QUIRC_ECC_LEVEL_L 1
#define K_QUIRC_ECC_LEVEL_H 2
#define K_QUIRC_ECC_LEVEL_Q 3

/* QR-code data types. */
#define K_QUIRC_DATA_TYPE_NUMERIC 1
#define K_QUIRC_DATA_TYPE_ALPHA 2
#define K_QUIRC_DATA_TYPE_BYTE 4
#define K_QUIRC_DATA_TYPE_KANJI 8

/* Decoder error codes */
typedef enum {
  K_QUIRC_SUCCESS = 0,
  K_QUIRC_ERROR_INVALID_GRID_SIZE,
  K_QUIRC_ERROR_INVALID_VERSION,
  K_QUIRC_ERROR_FORMAT_ECC,
  K_QUIRC_ERROR_DATA_ECC,
  K_QUIRC_ERROR_UNKNOWN_DATA_TYPE,
  K_QUIRC_ERROR_DATA_OVERFLOW,
  K_QUIRC_ERROR_DATA_UNDERFLOW,
  K_QUIRC_ERROR_ALLOC_FAILED,
} k_quirc_error_t;

/* Point structure for corners */
typedef struct {
  int x;
  int y;
} k_quirc_point_t;

/* This structure holds the decoded QR-code data */
typedef struct {
  int version;
  int ecc_level;
  int mask;
  int data_type;
  uint8_t payload[K_QUIRC_MAX_PAYLOAD];
  int payload_len;
  uint32_t eci;
} k_quirc_data_t;

/* QR code detection result */
typedef struct {
  k_quirc_point_t corners[4];
  k_quirc_data_t data;
  bool valid;
} k_quirc_result_t;

/* Opaque decoder context */
typedef struct k_quirc k_quirc_t;

/**
 * Create a new QR-code decoder instance.
 * @return Decoder instance or NULL on allocation failure
 */
k_quirc_t *k_quirc_new(void);

/**
 * Destroy a QR-code decoder instance and free all resources.
 * @param q Decoder instance (can be NULL)
 */
void k_quirc_destroy(k_quirc_t *q);

/**
 * Resize the decoder for a specific image size.
 * Must be called before decoding.
 * @param q Decoder instance
 * @param w Image width
 * @param h Image height
 * @return 0 on success, -1 on allocation failure
 */
int k_quirc_resize(k_quirc_t *q, int w, int h);

/**
 * Begin decoding - get pointer to grayscale image buffer.
 * Fill this buffer with grayscale image data before calling k_quirc_end().
 * @param q Decoder instance
 * @param w Optional pointer to receive width
 * @param h Optional pointer to receive height
 * @return Pointer to grayscale buffer
 */
uint8_t *k_quirc_begin(k_quirc_t *q, int *w, int *h);

/**
 * End decoding - process the image and detect QR codes.
 * @param q Decoder instance
 * @param find_inverted If true, also try to find inverted (white on black) QR
 * codes
 */
void k_quirc_end(k_quirc_t *q, bool find_inverted);

/**
 * Get the number of QR codes detected.
 * @param q Decoder instance
 * @return Number of detected QR codes
 */
int k_quirc_count(const k_quirc_t *q);

/**
 * Decode a specific QR code and get its data.
 * @param q Decoder instance
 * @param index QR code index (0 to k_quirc_count()-1)
 * @param result Pointer to result structure to fill
 * @return K_QUIRC_SUCCESS on success, error code otherwise
 */
k_quirc_error_t k_quirc_decode(k_quirc_t *q, int index,
                               k_quirc_result_t *result);

/**
 * Get a human-readable error message.
 * @param err Error code
 * @return Error message string
 */
const char *k_quirc_strerror(k_quirc_error_t err);

/**
 * Convenience function: Decode QR codes from grayscale image.
 * This combines resize, begin, end, and decode into a single call.
 *
 * @param grayscale_data Grayscale image data (8-bit per pixel)
 * @param width Image width
 * @param height Image height
 * @param results Array to store results (caller allocated)
 * @param max_results Maximum number of results to return
 * @param find_inverted If true, also try inverted QR codes
 * @return Number of QR codes successfully decoded
 */
int k_quirc_decode_grayscale(const uint8_t *grayscale_data, int width,
                             int height, k_quirc_result_t *results,
                             int max_results, bool find_inverted);

/* Effort level for the adaptive-threshold decode sweep. */
typedef enum {
  K_QUIRC_EFFORT_FAST = 0, /* bounded sweep — animated scans (bound the tax on
                              undecodable frames so throughput doesn't stall) */
  K_QUIRC_EFFORT_THOROUGH, /* full sweep — static scans (paper/metal/SeedQR),
                              where a dropped frame costs nothing */
} k_quirc_effort_t;

/* Per-call cost/behaviour readout for k_quirc_decode_adaptive (all optional).
 */
typedef struct {
  int passes; /* identify passes performed; the per-frame cost driver — 1
                 once locked, up to the ladder cap on acquisition */
  int locked_offset; /* threshold offset in effect at return (the lock on a hit,
                        the restored seed on a miss) */
  bool decoded;      /* a code decoded (mirrors the return value) */
} k_quirc_adaptive_stats_t;

/**
 * Adaptive-threshold decode with bootstrap sweep + lock.
 *
 * Fill the grayscale buffer via k_quirc_begin() first, then call this instead
 * of end()+count()+decode(). Sweeps the binarization threshold offset to find
 * one that decodes, seeded from the current (locked) offset, and LEAVES the
 * winning offset in place so the next (similar) frame decodes immediately at
 * ~one pass. Composes with the per-grid decode, including any corner-nudge
 * rescue.
 *
 * The threshold binarizes the image in place, so this snapshots the grayscale
 * once (in a buffer cached on the decoder) and restores it before each
 * re-identify.
 *
 * @param q       Decoder instance (already resized + filled via k_quirc_begin)
 * @param result  Receives the first decoded code
 * @param effort  FAST (bounded) or THOROUGH (full sweep)
 * @param stats   Optional (nullable) per-call cost/behaviour readout
 * @return 1 if a code decoded (into *result), 0 otherwise
 */
int k_quirc_decode_adaptive(k_quirc_t *q, k_quirc_result_t *result,
                            k_quirc_effort_t effort,
                            k_quirc_adaptive_stats_t *stats);

/* Media profile for the adaptive sweep's offset ladder. Emissive sources
 * (a code on an LCD/OLED) and reflective ones (paper/metal) want threshold
 * offsets in different regimes, so each profile is a different ladder. */
typedef enum {
  K_QUIRC_LADDER_EMISSIVE = 0, /* negative-leaning, reach -30: camera captures
                                  of bright emissive sources cluster at
                                  -10..-30. The first rungs are the original
                                  ladder's, so FAST behaviour at the default
                                  probe budget is unchanged. */
  K_QUIRC_LADDER_REFLECTIVE,   /* shallow +-10 flanks around the seed: in-focus
                                  reflective media decodes at or near the
                                  default threshold; deep rungs are wasted
                                  passes there. */
} k_quirc_ladder_t;

/**
 * Select the media-profile ladder k_quirc_decode_adaptive sweeps
 * (per-instance). A fresh decoder starts on K_QUIRC_LADDER_DEFAULT
 * (emissive; override the macro at build time). Callable in every build
 * configuration; a no-op when K_QUIRC_ADAPTIVE_THRESHOLD is compiled out.
 */
void k_quirc_set_ladder(k_quirc_t *q, k_quirc_ladder_t ladder);

/**
 * Numeric probe budget for k_quirc_decode_adaptive (per-instance).
 *
 * When set (> 0), overrides the effort level's built-in pass cap (FAST
 * K_QUIRC_FAST_CAP_DEFAULT / THOROUGH full ladder) with an explicit per-call
 * probe budget. 0 restores the effort defaults. The main use is opting a
 * parallel-decoder consumer into deeper animated-scan reach (e.g. cap 6
 * extends FAST through the emissive ladder's -25 rung) or tightening the
 * bound further for a slow single decoder. Callable in every build
 * configuration; a no-op when K_QUIRC_ADAPTIVE_THRESHOLD is compiled out.
 */
void k_quirc_set_sweep_cap(k_quirc_t *q, int cap);

/* Debug visualization support */
#ifdef K_QUIRC_DEBUG

#define K_QUIRC_DEBUG_MAX_GRIDS 8
#define K_QUIRC_DEBUG_MAX_CAPSTONES 32

#define K_QUIRC_PIXEL_WHITE 0
#define K_QUIRC_PIXEL_BLACK 1
#define K_QUIRC_PIXEL_REGION 2

typedef struct {
  int x;
  int y;
} k_quirc_debug_point_t;

typedef struct {
  float c[8];
  int grid_size;
  int timing_bias;
} k_quirc_debug_grid_t;

typedef struct {
  const void *pixels;
  int w, h;
  int num_grids;
  k_quirc_debug_grid_t grids[K_QUIRC_DEBUG_MAX_GRIDS];
  int num_capstones;
  k_quirc_debug_point_t capstones[K_QUIRC_DEBUG_MAX_CAPSTONES];
  int threshold_offset;
} k_quirc_debug_info_t;

const k_quirc_debug_info_t *k_quirc_get_debug_info(const k_quirc_t *q);

#endif /* K_QUIRC_DEBUG */

#endif /* K_QUIRC_H */
