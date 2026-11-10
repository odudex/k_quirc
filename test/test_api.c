#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "k_quirc.h"
#include "k_quirc_internal.h"

static int failures;

static void check_int(const char *name, int got, int want) {
  if (got == want)
    return;
  fprintf(stderr, "%s: got %d, want %d\n", name, got, want);
  failures++;
}

static void check_ptr_null(const char *name, const void *ptr) {
  if (!ptr)
    return;
  fprintf(stderr, "%s: got non-null pointer\n", name);
  failures++;
}

static void check_ptr_nonnull(const char *name, const void *ptr) {
  if (ptr)
    return;
  fprintf(stderr, "%s: got null pointer\n", name);
  failures++;
}

/* A fit whose denominator crosses zero inside the grid, or reaches it at a
 * corner, maps cells to infinity.  Sampling such a grid must read nothing
 * outside the image and report corners an int can hold; the conversions used
 * to be made first and checked after, which is undefined for what a
 * degenerate fit produces and only showed under -fsanitize=float-cast-overflow.
 */
static void test_degenerate_fit(void) {
  static const float fits[][QUIRC_PERSPECTIVE_PARAMS] = {
      {1, 0, 0, 0, 1, 0, -0.1f, 0},      /* zero at u = 10 */
      {1, 0, 0, 0, 1, 0, 0, -0.1f},      /* zero at v = 10 */
      {1, 0, 0, 0, 1, 0, -1.0f / 21, 0}, /* zero at the corner u = 21 */
      {1e30f, 0, 0, 0, 1e30f, 0, 0, 0},  /* finite, and off any image */
      {1, 0, 0, 0, 1, 0, -1.0f / 42, -1.0f / 42},
  };
  static k_quirc_result_t result;
  k_quirc_t *q = k_quirc_new();

  if (!q || k_quirc_resize(q, 64, 64) < 0) {
    fprintf(stderr, "degenerate fit: cannot set up a decoder\n");
    failures++;
    k_quirc_destroy(q);
    return;
  }

  for (size_t i = 0; i < sizeof(fits) / sizeof(fits[0]); i++) {
    uint8_t *image = k_quirc_begin(q, NULL, NULL);
    if (!image) {
      failures++;
      break;
    }
    memset(image, 0, 64 * 64);

    /* As identification would leave it, had it fitted this */
    q->num_grids = 1;
    q->grids[0].grid_size = 21;
    memcpy(q->grids[0].c, fits[i], sizeof(fits[i]));

    k_quirc_error_t err = k_quirc_decode(q, 0, &result);
    if (err == K_QUIRC_SUCCESS || result.valid) {
      fprintf(stderr, "degenerate fit %zu: decoded\n", i);
      failures++;
    }
    for (int corner = 0; corner < 4; corner++) {
      int x = result.corners[corner].x;
      int y = result.corners[corner].y;
      if (x < -QUIRC_COORD_LIMIT || x > QUIRC_COORD_LIMIT ||
          y < -QUIRC_COORD_LIMIT || y > QUIRC_COORD_LIMIT) {
        fprintf(stderr, "degenerate fit %zu: corner %d at (%d, %d)\n", i,
                corner, x, y);
        failures++;
      }
    }
  }
  k_quirc_destroy(q);
}

int main(void) {
  k_quirc_result_t result;
  uint8_t image[16 * 16] = {0};
  int w = -1;
  int h = -1;

  check_int("resize null", k_quirc_resize(NULL, 16, 16), -1);
  check_ptr_null("begin null", k_quirc_begin(NULL, &w, &h));
  check_int("begin null width", w, 0);
  check_int("begin null height", h, 0);
  k_quirc_end(NULL, false);
  check_int("count null", k_quirc_count(NULL), 0);
  check_int("decode null q", k_quirc_decode(NULL, 0, &result),
            K_QUIRC_ERROR_INVALID_GRID_SIZE);
  check_int("decode null result", k_quirc_decode(NULL, 0, NULL),
            K_QUIRC_ERROR_INVALID_GRID_SIZE);
  check_int("decode grayscale null image",
            k_quirc_decode_grayscale(NULL, 16, 16, &result, 1, false), 0);
  check_int("decode grayscale null results",
            k_quirc_decode_grayscale(image, 16, 16, NULL, 1, false), 0);
  check_int("decode grayscale zero max",
            k_quirc_decode_grayscale(image, 16, 16, &result, 0, false), 0);

  k_quirc_t *q = k_quirc_new();
  check_ptr_nonnull("new", q);
  if (q) {
    check_int("resize valid", k_quirc_resize(q, 16, 16), 0);
    check_ptr_nonnull("begin valid", k_quirc_begin(q, &w, &h));
    check_int("begin valid width", w, 16);
    check_int("begin valid height", h, 16);

    check_int("resize invalid preserves failure", k_quirc_resize(q, -1, 16),
              -1);
    check_ptr_nonnull("begin after invalid resize", k_quirc_begin(q, &w, &h));
    check_int("width after invalid resize", w, 16);
    check_int("height after invalid resize", h, 16);

    k_quirc_set_threshold_offset_for(q, 99);
    check_int("threshold clamp high", k_quirc_get_threshold_offset_for(q),
              K_QUIRC_THRESHOLD_OFFSET_MAX);
    k_quirc_set_threshold_offset_for(q, -99);
    check_int("threshold clamp low", k_quirc_get_threshold_offset_for(q),
              -K_QUIRC_THRESHOLD_OFFSET_MAX);

    k_quirc_destroy(q);
  }

  k_quirc_set_threshold_offset(3);
  k_quirc_t *q1 = k_quirc_new();
  k_quirc_set_threshold_offset(11);
  k_quirc_t *q2 = k_quirc_new();
  if (q1 && q2) {
    check_int("threshold instance q1", k_quirc_get_threshold_offset_for(q1), 3);
    check_int("threshold instance q2", k_quirc_get_threshold_offset_for(q2),
              11);
  } else {
    fprintf(stderr, "failed to allocate threshold test decoders\n");
    failures++;
  }
  k_quirc_destroy(q1);
  k_quirc_destroy(q2);
  k_quirc_set_threshold_offset(K_QUIRC_THRESHOLD_OFFSET_DEFAULT);

  test_degenerate_fit();

  return failures ? 1 : 0;
}
