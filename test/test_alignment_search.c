/* Near-parallel capstone edges put the alignment estimate far outside the
 * image, and the search area overflowed into a billion-pixel spiral.  Values
 * from an ESP32-P4 watchdog dump: 600x600 frame, estimate at (8678, 250901),
 * a module of 9842 x 9842 pixels. */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "../src/k_quirc_identify.c"

/* Affine "perspective": every module is scale x scale pixels. */
static void set_scale(struct quirc_capstone *capstone, float scale) {
  memset(capstone, 0, sizeof(*capstone));
  capstone->c[0] = scale;
  capstone->c[4] = scale;
}

static int failures;

static void check_search(const char *name, int align_x, int align_y,
                         float scale) {
  k_quirc_t *q = k_quirc_new();
  if (!q || k_quirc_resize(q, 600, 600) < 0) {
    printf("FAIL %s: decoder setup\n", name);
    failures++;
    return;
  }

  set_scale(&q->capstones[0], scale);
  set_scale(&q->capstones[1], scale);
  q->num_capstones = 2;
  struct quirc_grid *grid = &q->grids[0];
  memset(grid, 0, sizeof(*grid));
  grid->caps[0] = 0;
  grid->caps[2] = 1;
  grid->align.x = align_x;
  grid->align.y = align_y;
  grid->align_region = -1;

  clock_t start = clock();
  find_alignment_pattern(q, 0);
  double seconds = (double)(clock() - start) / CLOCKS_PER_SEC;

  /* Unbounded, this takes seconds even on a desktop */
  if (seconds > 0.25 || grid->align_region != -1) {
    printf("FAIL %s: %.2f s, align_region %d\n", name, seconds,
           grid->align_region);
    failures++;
  } else {
    printf("ok   %s (%.4f s)\n", name, seconds);
  }
  k_quirc_destroy(q);
}

int main(void) {
  check_search("estimate far outside the image, module of 96.9M px", 8678,
               250901, 9842.0f);
  check_search("estimate inside the image, module of 96.9M px", 300, 300,
               9842.0f);
  check_search("module area overflows a 32-bit product", 300, 300, 60000.0f);
  check_search("module just over a hundredth of the image", 300, 300, 61.0f);
  check_search("coordinates beyond the integer range", 2000000000, -2000000000,
               3.0e9f);

  if (failures) {
    printf("%d alignment search test(s) failed\n", failures);
    return 1;
  }
  puts("All alignment search tests passed.");
  return 0;
}
