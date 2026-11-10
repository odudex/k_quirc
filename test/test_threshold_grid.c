/*
 * White-level mesh of one block (K_QUIRC_THRESHOLD_GRID=1) on a large frame.
 *
 * The mesh measures each block's white level from a histogram with 16-bit
 * bins.  Derived from the image, a block takes a few thousand samples.  Fixed
 * at one block it takes the whole frame's, and past 768 pixels a side an
 * evenly lit page put more than 65,535 of them in one bin.  Where the count
 * of white samples wrapped to nearly nothing the block seemed to have no
 * white at all, its white level was taken from the next thing down, and the
 * threshold was scaled to that.
 *
 * Whether a count wraps to nearly nothing depends on how much of the frame is
 * white, so the share is swept.  The frame is three flat bands: dark ink, a
 * mid-gray well below the threshold, and white.  The gray band must binarize
 * as ink however wide it is.
 */
#include <stdio.h>
#include <string.h>

#include "../src/k_quirc_identify.c"

#if K_QUIRC_THRESHOLD_GRID != 1
#error "build this test with K_QUIRC_THRESHOLD_GRID=1"
#endif

#define SIZE 1024
#define INK 40
#define GRAY 100
#define WHITE 250
#define THRESHOLD 145 /* midway between ink and white, above the gray */
#define INK_ROWS 64
#define MAX_GRAY_ROWS 800 /* leaves a seventh of the frame white */

int main(void) {
  k_quirc_t *q = k_quirc_new();
  int failures = 0;

  if (!q || k_quirc_resize(q, SIZE, SIZE) < 0 || sizeof(*q->pixels) != 1) {
    printf("FAIL decoder setup\n");
    k_quirc_destroy(q);
    return 1;
  }

  /* One sampled row more of gray at a time, whatever the sampling step */
  for (int gray_rows = 1; gray_rows <= MAX_GRAY_ROWS; gray_rows++) {
    memset(q->pixels, WHITE, (size_t)SIZE * SIZE);
    memset(q->pixels, INK, (size_t)SIZE * INK_ROWS);
    memset(q->pixels + (size_t)SIZE * INK_ROWS, GRAY, (size_t)SIZE * gray_rows);

    binarize_by_white_level(q, THRESHOLD, WHITE >> THRESHOLD_BIN_SHIFT, 0);

    int ink = q->pixels[(size_t)SIZE * (INK_ROWS / 2) + SIZE / 2];
    int gray = q->pixels[(size_t)SIZE * (INK_ROWS + gray_rows / 2) + SIZE / 2];
    int white = q->pixels[(size_t)SIZE * (SIZE - 1) + SIZE / 2];
    if (ink != QUIRC_PIXEL_BLACK || gray != QUIRC_PIXEL_BLACK ||
        white != QUIRC_PIXEL_WHITE) {
      if (failures++ < 5)
        printf("FAIL %d gray rows: ink %d, gray %d, white %d\n", gray_rows, ink,
               gray, white);
    }
  }
  k_quirc_destroy(q);

  if (failures) {
    printf("%d of %d frames binarized wrongly\n", failures, MAX_GRAY_ROWS);
    return 1;
  }
  printf("All %d frames binarized as their levels say.\n", MAX_GRAY_ROWS);
  return 0;
}
