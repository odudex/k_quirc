/*
 * Two-level (strictly bimodal) image regression test.
 *
 * A screen-rendered or freshly generated QR has exactly two grey levels and an
 * empty histogram between them.  Every threshold inside that gap produces the
 * same partition and therefore the same between-class variance, so Otsu's
 * argmax is a plateau, not a point.  Picking either end of the plateau parks
 * the threshold flush against one of the two modes, and the adaptive offset
 * then pushes it across -- the whole image binarizes to a single colour and no
 * finder pattern survives.
 *
 * Only light == 255 escaped, because the offset clamped at 255 and the
 * comparison is `p < t`.  That is why the pure-black-on-white fixtures kept
 * passing.  This test sweeps the light level so the failure cannot hide.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_LINEAR
#define STBI_NO_HDR
#include "stb_image.h"

#include "k_quirc.h"

#ifndef BIMODAL_VECTOR
#define BIMODAL_VECTOR "vectors/cv_v06_ppm08_sig000_ev00_ink00_gr00_tilt00.png"
#endif

/* Dark level, light level.  The dark level is held at a plausible ink value; a
 * pure 0 would only make the gap wider and the test easier. */
#define DARK 51

static const int light_levels[] = {96, 128, 160, 195, 220, 246, 254, 255};
#define NUM_LEVELS ((int)(sizeof(light_levels) / sizeof(light_levels[0])))

/* Collapse a grayscale image onto two levels, as a screen or a synthetic
 * renderer would produce it. */
static void quantize(uint8_t *dst, const uint8_t *src, size_t n, int light) {
  for (size_t i = 0; i < n; i++)
    dst[i] = (src[i] < 128) ? (uint8_t)DARK : (uint8_t)light;
}

/* One frame through a fresh context, so the adaptive offset is at its default
 * -- which is exactly the configuration that failed on the device. */
static int decodes(const uint8_t *image, int w, int h) {
  k_quirc_t *q = k_quirc_new();
  if (!q)
    return -1;
  if (k_quirc_resize(q, w, h) < 0) {
    k_quirc_destroy(q);
    return -1;
  }

  uint8_t *buf = k_quirc_begin(q, NULL, NULL);
  if (!buf) {
    k_quirc_destroy(q);
    return -1;
  }
  memcpy(buf, image, (size_t)w * (size_t)h);
  k_quirc_end(q, false);

  int decoded = 0;
  int count = k_quirc_count(q);
  for (int i = 0; i < count; i++) {
    k_quirc_result_t result;
    if (k_quirc_decode(q, i, &result) == K_QUIRC_SUCCESS) {
      decoded = 1;
      break;
    }
  }

  k_quirc_destroy(q);
  return decoded;
}

int main(int argc, char **argv) {
  const char *path = (argc > 1) ? argv[1] : BIMODAL_VECTOR;

  int w, h, channels;
  uint8_t *src = stbi_load(path, &w, &h, &channels, 1);
  if (!src) {
    fprintf(stderr, "error: cannot load '%s'\n", path);
    return 1;
  }

  size_t n = (size_t)w * (size_t)h;
  uint8_t *frame = malloc(n);
  if (!frame) {
    stbi_image_free(src);
    return 1;
  }

  printf("Two-level sweep on %s (%dx%d), dark=%d\n\n", path, w, h, DARK);
  printf("  light  decoded\n");
  printf("  -----  -------\n");

  int failures = 0;
  for (int i = 0; i < NUM_LEVELS; i++) {
    int light = light_levels[i];
    quantize(frame, src, n, light);

    int ok = decodes(frame, w, h);
    printf("  %5d  %s\n", light, ok > 0 ? "yes" : "NO");
    if (ok <= 0)
      failures++;
  }

  free(frame);
  stbi_image_free(src);

  printf("\n%d/%d levels decoded\n", NUM_LEVELS - failures, NUM_LEVELS);
  if (failures) {
    printf("FAIL: %d light level(s) did not decode\n", failures);
    return 1;
  }
  printf("PASS\n");
  return 0;
}
