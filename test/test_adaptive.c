/*
 * test_adaptive.c -- exercises the k_quirc_decode_adaptive() bootstrap sweep +
 * lock on a synthetic frame the fixed threshold path CANNOT decode.
 *
 * Why synthetic (and why blurred + dark surround): k_quirc binarizes with
 * threshold = Otsu + offset. A clean two-level QR decodes at every offset in a
 * wide band, so it can never exercise the sweep (see
 * docs/knowledge/qr-threshold-test-image-synthesis.md in the builder repo).
 * Offset sensitivity needs (a) anti-aliased edges putting pixel mass INSIDE the
 * histogram valley -- a blur -- and (b) an asymmetric bright/dark surround mass
 * that drags Otsu off the module midpoint. That is exactly the emissive-LCD
 * bloom regime the sweep was built for: a bright QR on a large surround pins
 * the histogram high, so the code only binarizes cleanly at a NEGATIVE offset.
 *
 * The QR module matrix below is baked from qrencode (v2, ECC-L,
 * "SEEDSIGNER-ADAPTIVE-TEST-1234567890") so the test needs no binary asset and
 * no build-time tooling; it is rendered in-process with faded gray levels and a
 * cheap separable box blur. The resulting frame fails at the default +10 offset
 * but decodes across roughly -20..+5, so the sweep rescues it and locks a
 * negative offset; a second (identical) frame then decodes in a single pass.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "k_quirc.h"
#include "k_quirc_internal.h"

#ifndef K_QUIRC_ADAPTIVE_THRESHOLD
/* The sweep + lock only exists when the adaptive threshold is compiled in;
 * without it k_quirc_decode_adaptive degrades to a single fixed-offset decode
 * (nothing offset-specific to assert). Skip so the harness still builds and
 * links in the flag-off configuration. */
int main(void) {
  printf("test_adaptive: SKIP (built without K_QUIRC_ADAPTIVE_THRESHOLD)\n");
  return 0;
}
#else

/* --- baked QR module matrix (bit x of row y set => dark module) --- */
#define QR_MODULES 25
static const uint32_t QR_BITS[QR_MODULES] = {
    0x01fd6f7fu, 0x01052941u, 0x0174e25du, 0x0174985du, 0x0174f95du,
    0x01045741u, 0x01fd557fu, 0x00004300u, 0x019f7b67u, 0x0029b124u,
    0x01901767u, 0x00945c8bu, 0x010d075au, 0x017925acu, 0x00868967u,
    0x01cb8200u, 0x005f38dbu, 0x00319500u, 0x0015127fu, 0x00111941u,
    0x001fd25du, 0x0050a45du, 0x01a15b5du, 0x014a8141u, 0x01defb7fu,
};
static const char EXPECTED_PAYLOAD[] = "SEEDSIGNER-ADAPTIVE-TEST-1234567890";

/* Render parameters (tuned so the frame fails at +10 but decodes negative). */
#define FRAME 320
#define SCALE 8    /* px per module -> 200 px QR */
#define DARK 90    /* faded dark modules (not full black) */
#define LIGHT 245  /* bright light modules */
#define CANVAS 180 /* large mid-bright surround -> drags Otsu up */
#define BLUR_RADIUS 1
#define BLUR_PASSES 1

static void render_frame(uint8_t *img) {
  const int qs = QR_MODULES * SCALE;
  const int off = (FRAME - qs) / 2;
  for (int i = 0; i < FRAME * FRAME; i++)
    img[i] = (uint8_t)CANVAS;
  for (int my = 0; my < QR_MODULES; my++)
    for (int mx = 0; mx < QR_MODULES; mx++) {
      int val = ((QR_BITS[my] >> mx) & 1u) ? DARK : LIGHT;
      for (int dy = 0; dy < SCALE; dy++)
        for (int dx = 0; dx < SCALE; dx++)
          img[(off + my * SCALE + dy) * FRAME + (off + mx * SCALE + dx)] =
              (uint8_t)val;
    }
  /* Separable box blur (BLUR_PASSES times) fills the histogram valley with
   * gradient edge pixels so each offset step reclassifies real area. */
  uint8_t *tmp = malloc((size_t)FRAME * FRAME);
  if (!tmp)
    return;
  for (int p = 0; p < BLUR_PASSES; p++) {
    for (int y = 0; y < FRAME; y++)
      for (int x = 0; x < FRAME; x++) {
        int s = 0, cnt = 0;
        for (int k = -BLUR_RADIUS; k <= BLUR_RADIUS; k++) {
          int xx = x + k;
          if (xx < 0 || xx >= FRAME)
            continue;
          s += img[y * FRAME + xx];
          cnt++;
        }
        tmp[y * FRAME + x] = (uint8_t)(s / cnt);
      }
    for (int y = 0; y < FRAME; y++)
      for (int x = 0; x < FRAME; x++) {
        int s = 0, cnt = 0;
        for (int k = -BLUR_RADIUS; k <= BLUR_RADIUS; k++) {
          int yy = y + k;
          if (yy < 0 || yy >= FRAME)
            continue;
          s += tmp[yy * FRAME + x];
          cnt++;
        }
        img[y * FRAME + x] = (uint8_t)(s / cnt);
      }
  }
  free(tmp);
}

static int failures;
static void fail(const char *msg) {
  fprintf(stderr, "FAIL: %s\n", msg);
  failures++;
}

/* Fixed-offset decode at a given offset (no sweep). Returns 1 if decoded. */
static int decode_fixed(k_quirc_t *q, const uint8_t *img, int offset) {
  k_quirc_set_threshold_offset_for(q, offset);
  uint8_t *buf = k_quirc_begin(q, NULL, NULL);
  memcpy(buf, img, (size_t)FRAME * FRAME);
  k_quirc_end(q, false);
  k_quirc_result_t r;
  for (int g = 0; g < k_quirc_count(q); g++)
    if (k_quirc_decode(q, g, &r) == K_QUIRC_SUCCESS && r.valid)
      return 1;
  return 0;
}

int main(void) {
  uint8_t *img = malloc((size_t)FRAME * FRAME);
  if (!img) {
    fprintf(stderr, "alloc failed\n");
    return 1;
  }
  render_frame(img);

  k_quirc_t *q = k_quirc_new();
  if (!q || k_quirc_resize(q, FRAME, FRAME) < 0) {
    fprintf(stderr, "decoder init failed\n");
    return 1;
  }

  /* 1. The fixed default (+10) path must NOT decode this frame -- that is the
   *    whole point of the corpus population the sweep targets. */
  if (decode_fixed(q, img, K_QUIRC_THRESHOLD_OFFSET_DEFAULT))
    fail("fixed +10 path unexpectedly decoded the faded frame");

  /* Fresh decoder so the lock starts cold at the default seed, as on a live
   * scan's first located frame. */
  k_quirc_destroy(q);
  q = k_quirc_new();
  k_quirc_resize(q, FRAME, FRAME);

  /* 2. The adaptive sweep (THOROUGH) must decode it, recover the payload, and
   *    lock a negative offset (below the +10 seed) using >1 pass (acquisition).
   */
  uint8_t *buf = k_quirc_begin(q, NULL, NULL);
  memcpy(buf, img, (size_t)FRAME * FRAME);
  k_quirc_result_t r;
  k_quirc_adaptive_stats_t st;
  int ok = k_quirc_decode_adaptive(q, &r, K_QUIRC_EFFORT_THOROUGH, &st);
  if (!ok)
    fail("adaptive sweep failed to decode the faded frame");
  if (ok && !r.valid)
    fail("adaptive returned success but result marked invalid");
  if (ok && strcmp((const char *)r.data.payload, EXPECTED_PAYLOAD) != 0)
    fail("adaptive decoded the wrong payload");
  if (ok && st.locked_offset >= K_QUIRC_THRESHOLD_OFFSET_DEFAULT)
    fail("adaptive did not lock a lower (negative-regime) offset");
  if (ok && st.passes < 2)
    fail("acquisition should take more than one pass (seed then ladder)");
  int locked = st.locked_offset;

  /* 3. Lock persistence: a second, identical frame must now decode in a single
   *    pass because the winning offset is already seeded. */
  buf = k_quirc_begin(q, NULL, NULL);
  memcpy(buf, img, (size_t)FRAME * FRAME);
  ok = k_quirc_decode_adaptive(q, &r, K_QUIRC_EFFORT_THOROUGH, &st);
  if (!ok)
    fail("locked decoder failed to decode the second frame");
  if (ok && st.passes != 1)
    fail("locked steady-state frame should decode in exactly one pass");
  if (ok && st.locked_offset != locked)
    fail("lock offset drifted between identical frames");

  k_quirc_destroy(q);
  free(img);

  if (failures) {
    fprintf(stderr, "%d assertion(s) failed\n", failures);
    return 1;
  }
  printf("test_adaptive: OK (fixed +10 miss; sweep locked offset %d; "
         "steady state 1 pass)\n",
         locked);
  return 0;
}
#endif /* K_QUIRC_ADAPTIVE_THRESHOLD */
