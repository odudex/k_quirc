/*
 * Uneven light regression test.
 *
 * A screen read at an angle through an uncorrected lens falls off towards one
 * corner of the frame: a capture measured white at 232 in the centre and 110
 * in the code's far corner.  A threshold taken from the centre then sits at
 * that corner's white level, the corner binarizes solid dark and the read is
 * lost, the same parts of an animated sequence every time round.
 *
 * Each vector is dimmed towards each corner in turn, faster the nearer the
 * corner as vignetting is, and run through one context for a few frames, as a
 * scanner would hold on it while the threshold loop settles.
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

#ifndef FALLOFF_VECTORS_DIR
#define FALLOFF_VECTORS_DIR "convergence/vectors"
#endif

static const char *const vectors[] = {
    "cv_v06_ppm08_sig000_ev00_ink00_gr00_tilt00.png",
    "cv_v10_ppm06_sig090_ev05_ink00_gr00_tilt16.png",
    "cv_v14_ppm05_sig090_ev05_ink00_gr00_tilt14.png",
};
#define NUM_VECTORS ((int)(sizeof(vectors) / sizeof(vectors[0])))

/* Light left at the frame's dim corner, percent: about 45 at the code's */
#define FAR_CORNER_LIGHT 25
#define FRAMES 8

/* Scale by 1 - (1 - far) d^3, d running 0 to 1 across the frame's diagonal
 * towards the dim corner */
static void dim_towards(uint8_t *dst, const uint8_t *src, int w, int h,
                        int corner) {
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int dx = (corner & 1) ? x : w - 1 - x;
      int dy = (corner & 2) ? y : h - 1 - y;
      int d = (dx * 512 / (w - 1) + dy * 512 / (h - 1)); /* 0..1024 */
      int light =
          1024 * 100 - (100 - FAR_CORNER_LIGHT) * ((d * d / 1024) * d / 1024);
      dst[y * w + x] = (uint8_t)(src[y * w + x] * light / (1024 * 100));
    }
  }
}

/* Frames until the first decode, or 0 */
static int frames_to_decode(const uint8_t *image, int w, int h) {
  k_quirc_t *q = k_quirc_new();
  int locked = 0;

  if (!q || k_quirc_resize(q, w, h) < 0) {
    k_quirc_destroy(q);
    return -1;
  }
  for (int frame = 1; frame <= FRAMES && !locked; frame++) {
    uint8_t *buf = k_quirc_begin(q, NULL, NULL);
    if (!buf)
      break;
    memcpy(buf, image, (size_t)w * (size_t)h);
    k_quirc_end(q, false);

    int count = k_quirc_count(q);
    for (int i = 0; i < count; i++) {
      k_quirc_result_t result;
      if (k_quirc_decode(q, i, &result) == K_QUIRC_SUCCESS) {
        locked = frame;
        break;
      }
    }
  }
  k_quirc_destroy(q);
  return locked;
}

int main(int argc, char **argv) {
  const char *dir = (argc > 1) ? argv[1] : FALLOFF_VECTORS_DIR;
  static const char *const corners[] = {"top-left", "top-right", "bottom-left",
                                        "bottom-right"};
  int failures = 0;

  printf("Light falling to %d%% at one corner, %d frames per run\n\n",
         FAR_CORNER_LIGHT, FRAMES);
  for (int v = 0; v < NUM_VECTORS; v++) {
    char path[1024];
    int w, h, channels;

    snprintf(path, sizeof(path), "%s/%s", dir, vectors[v]);
    uint8_t *src = stbi_load(path, &w, &h, &channels, 1);
    uint8_t *frame = src ? malloc((size_t)w * (size_t)h) : NULL;
    if (!frame) {
      fprintf(stderr, "error: cannot load '%s'\n", path);
      stbi_image_free(src);
      return 1;
    }

    printf("  %s\n", vectors[v]);
    for (int corner = 0; corner < 4; corner++) {
      dim_towards(frame, src, w, h, corner);
      int locked = frames_to_decode(frame, w, h);
      if (locked > 0) {
        printf("    dim %-13s decoded on frame %d\n", corners[corner], locked);
      } else {
        printf("    dim %-13s NOT decoded\n", corners[corner]);
        failures++;
      }
    }
    free(frame);
    stbi_image_free(src);
  }

  if (failures) {
    printf("\nFAIL: %d run(s) did not decode\n", failures);
    return 1;
  }
  printf("\nPASS\n");
  return 0;
}
