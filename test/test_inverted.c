/*
 * Inverted (light-on-dark) QR regression test.
 *
 * K_QUIRC_INVERTED_RETRY reads a frame's negative when the frame itself shows
 * no code.  The retry used to threshold the image a second time, and by then
 * there was no image: the region labels share its buffer, so what it
 * thresholded was the first pass's binarization.  Only builds with 16-bit
 * labels, which keep the two apart, ever read an inverted code - and those
 * only where black and white straddled mid-gray, because the threshold was
 * applied to the negated pixels without being negated itself.
 *
 * Nothing else in the suite holds a light-on-dark code, so the vectors here
 * are their own: dark-themed screens and dark labels, sharp and through the
 * camera model, with levels that do and do not straddle mid-gray.  The test is
 * built once per label width.
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

#ifndef INVERTED_VECTORS_DIR
#define INVERTED_VECTORS_DIR "inverted/vectors"
#endif
#ifndef NORMAL_VECTOR
#define NORMAL_VECTOR                                                          \
  "convergence/vectors/cv_v06_ppm08_sig000_ev00_ink00_gr00_tilt00.png"
#endif
#define NORMAL_VERSION 6

static const struct {
  const char *name;
  int version;
} vectors[] = {
    {"inv_v01_ppm10_sig000_ev00_gr00_tilt00_bg000_lv000-255.png", 1},
    {"inv_v06_ppm08_sig000_ev00_gr00_tilt00_bg032_lv030-110.png", 6},
    {"inv_v06_ppm08_sig000_ev00_gr00_tilt00_bg160_lv140-235.png", 6},
    {"inv_v06_ppm06_sig090_ev00_gr00_tilt12_bg032_lv000-255.png", 6},
    {"inv_v10_ppm06_sig090_ev05_gr00_tilt16_bg032_lv000-255.png", 10},
    {"inv_v10_ppm05_sig120_ev05_gr00_tilt15_bg032_lv000-255.png", 10},
    {"inv_v10_ppm06_sig090_ev00_gr25_tilt14_bg032_lv000-255.png", 10},
    {"inv_v14_ppm05_sig060_ev00_gr00_tilt14_bg128_lv000-255.png", 14},
};
#define NUM_VECTORS ((int)(sizeof(vectors) / sizeof(vectors[0])))

/* A scanner holds on a code for several frames */
#define FRAMES 3

static int failures;

static void check(int ok, const char *vector, const char *what) {
  if (ok)
    return;
  printf("    FAIL %s: %s\n", vector, what);
  failures++;
}

/* The payload every vector of a version carries; see gen_vectors.py.  What
 * does not fit `out` is left off, and then matches no vector. */
static int expected_payload(int version, char *out, size_t size) {
  static const char unit[] = "ABCDEFGHIJ";
  int repeats = (version >= 4) ? version * 2 : version;
  int prefix = snprintf(out, size, "V%02d:", version);
  size_t len = (prefix < 0 || (size_t)prefix >= size) ? 0 : (size_t)prefix;

  for (int i = 0; i < repeats && len + sizeof(unit) <= size; i++) {
    memcpy(out + len, unit, sizeof(unit)); /* the NUL with it */
    len += sizeof(unit) - 1;
  }
  return (int)len;
}

static bool is_payload(const k_quirc_result_t *result, int version) {
  char want[K_QUIRC_MAX_PAYLOAD];
  int len = expected_payload(version, want, sizeof(want));

  return result->valid && result->data.version == version &&
         result->data.payload_len == len &&
         memcmp(result->data.payload, want, (size_t)len) == 0;
}

/* One frame through a context: 1 if it read the version's payload, 0 if it
 * read nothing, -1 if it read anything else */
static int read_frame(k_quirc_t *q, const uint8_t *image, int w, int h,
                      bool find_inverted, int version) {
  static k_quirc_result_t result;
  uint8_t *buf = k_quirc_begin(q, NULL, NULL);

  if (!buf)
    return -1;
  memcpy(buf, image, (size_t)w * (size_t)h);
  k_quirc_end(q, find_inverted);

  int count = k_quirc_count(q);
  int read = 0;
  for (int i = 0; i < count; i++) {
    if (k_quirc_decode(q, i, &result) != K_QUIRC_SUCCESS)
      continue;
    if (!is_payload(&result, version))
      return -1;
    read = 1;
  }
  return read;
}

static uint8_t *load(const char *path, int *w, int *h) {
  int channels;
  uint8_t *image = stbi_load(path, w, h, &channels, 1);

  if (!image)
    fprintf(stderr, "error: cannot load '%s'\n", path);
  return image;
}

int main(int argc, char **argv) {
  const char *dir = (argc > 1) ? argv[1] : INVERTED_VECTORS_DIR;
  const char *normal_path = (argc > 2) ? argv[2] : NORMAL_VECTOR;
  static k_quirc_result_t result;
  int nw, nh;
  uint8_t *normal = load(normal_path, &nw, &nh);

  if (!normal)
    return 1;

  printf("Light-on-dark codes, %d frames per context\n\n", FRAMES);
  for (int v = 0; v < NUM_VECTORS; v++) {
    const char *name = vectors[v].name;
    int version = vectors[v].version;
    char path[1024];
    int w, h;

    snprintf(path, sizeof(path), "%s/%s", dir, name);
    uint8_t *image = load(path, &w, &h);
    if (!image)
      return 1;

    int before = failures;
    k_quirc_t *q = k_quirc_new();
    if (!q || k_quirc_resize(q, w, h) < 0) {
      fprintf(stderr, "error: cannot set up a %dx%d decoder\n", w, h);
      return 1;
    }

    /* Unasked, the negative is not read: a caller that does not expect
     * inverted codes is not handed one */
    check(read_frame(q, image, w, h, false, version) == 0, name,
          "read without find_inverted");

    for (int frame = 0; frame < FRAMES; frame++)
      check(read_frame(q, image, w, h, true, version) == 1, name,
            "not read with find_inverted");

    /* The retry leaves nothing behind for the frames that follow, of either
     * polarity */
    if (w == nw && h == nh) {
      check(read_frame(q, normal, w, h, true, NORMAL_VERSION) == 1, name,
            "dark-on-light frame not read after an inverted one");
      check(read_frame(q, image, w, h, true, version) == 1, name,
            "not read after a dark-on-light frame");
    }
    k_quirc_destroy(q);

    check(k_quirc_decode_grayscale(image, w, h, &result, 1, true) == 1 &&
              is_payload(&result, version),
          name, "not read by k_quirc_decode_grayscale");
    check(k_quirc_decode_grayscale(image, w, h, &result, 1, false) == 0, name,
          "read by k_quirc_decode_grayscale without find_inverted");

    printf("  %-60s %s\n", name, failures == before ? "ok" : "FAILED");
    stbi_image_free(image);
  }
  stbi_image_free(normal);

  if (failures) {
    printf("\nFAIL: %d check(s)\n", failures);
    return 1;
  }
  printf("\nPASS\n");
  return 0;
}
