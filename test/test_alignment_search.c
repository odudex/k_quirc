/* The alignment pattern search, reached through the static function.
 *
 * Near-parallel finder edges once put the estimate far outside the image and
 * sent a search across a billion pixels (ESP32-P4 watchdog dump: 600x600
 * frame, estimate at (8678, 250901)).  Whatever it is handed, the search must
 * stay inside the image and leave a hopeless estimate alone; handed a pattern
 * a few modules from the estimate, among look-alike modules, it must find it.
 */
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "../src/k_quirc_identify.c"

#define SIZE 600
#define PITCH 6

static int failures;

/* Three finders of `edge` pixels a side where a version 18 symbol of PITCH
 * pixels a module would have them; `corner_edge` is the corner finder's. */
static void place_finders(k_quirc_t *q, struct quirc_grid *grid, int edge,
                          int corner_edge) {
  static const int origin[3][2] = {
      {30, 30 + 82 * PITCH}, {30, 30}, {30 + 82 * PITCH, 30}};

  memset(grid, 0, sizeof(*grid));
  grid->grid_size = 89;
  grid->align_region = -1;
  q->num_capstones = 3;
  for (int i = 0; i < 3; i++) {
    struct quirc_capstone *cap = &q->capstones[i];
    int e = (i == 1) ? corner_edge : edge;
    memset(cap, 0, sizeof(*cap));
    cap->corners[0] = (struct quirc_point){origin[i][0], origin[i][1]};
    cap->corners[1] = (struct quirc_point){origin[i][0] + e, origin[i][1]};
    cap->corners[2] = (struct quirc_point){origin[i][0] + e, origin[i][1] + e};
    cap->corners[3] = (struct quirc_point){origin[i][0], origin[i][1] + e};
    grid->caps[i] = i;
  }
}

static void fill_module(k_quirc_t *q, int mx, int my, quirc_pixel_t value) {
  for (int y = 0; y < PITCH; y++)
    for (int x = 0; x < PITCH; x++)
      q->pixels[(30 + my * PITCH + y) * SIZE + 30 + mx * PITCH + x] = value;
}

static k_quirc_t *new_decoder(void) {
  k_quirc_t *q = k_quirc_new();
  if (!q || k_quirc_resize(q, SIZE, SIZE) < 0) {
    printf("FAIL decoder setup\n");
    failures++;
    k_quirc_destroy(q);
    return NULL;
  }
  memset(q->pixels, QUIRC_PIXEL_WHITE,
         (size_t)SIZE * SIZE * sizeof(*q->pixels));
  q->num_regions = QUIRC_PIXEL_REGION;
  return q;
}

/* The search must return at once and leave the estimate as it was */
static void check_left_alone(const char *name, int align_x, int align_y,
                             int edge, int corner_edge) {
  k_quirc_t *q = new_decoder();
  if (!q)
    return;

  struct quirc_grid *grid = &q->grids[0];
  place_finders(q, grid, edge, corner_edge);
  grid->align.x = align_x;
  grid->align.y = align_y;

  clock_t start = clock();
  find_alignment_pattern(q, 0);
  double seconds = (double)(clock() - start) / CLOCKS_PER_SEC;

  if (seconds > 0.25 || grid->align_region != -1 || grid->align.x != align_x ||
      grid->align.y != align_y) {
    printf("FAIL %s: %.2f s, align (%d,%d) region %d\n", name, seconds,
           grid->align.x, grid->align.y, grid->align_region);
    failures++;
  } else {
    printf("ok   %s (%.4f s)\n", name, seconds);
  }
  k_quirc_destroy(q);
}

/* A pattern centred on module (82, 82), lone dark modules all around it, and
 * an estimate `off` modules out: the first region of a module's size a search
 * outwards from there meets is one of the look-alikes. */
static void check_found(int off_x, int off_y) {
  k_quirc_t *q = new_decoder();
  if (!q)
    return;

  struct quirc_grid *grid = &q->grids[0];
  place_finders(q, grid, 7 * PITCH, 7 * PITCH);
  for (int my = 72; my < 89; my++)
    for (int mx = 72; mx < 89; mx++)
      if ((mx % 2 == 0) && (my % 2 == 0))
        fill_module(q, mx, my, QUIRC_PIXEL_BLACK);
  for (int my = 79; my <= 85; my++)
    for (int mx = 79; mx <= 85; mx++)
      fill_module(q, mx, my, QUIRC_PIXEL_WHITE);
  for (int my = 80; my <= 84; my++)
    for (int mx = 80; mx <= 84; mx++) {
      int ring = abs(mx - 82) > abs(my - 82) ? abs(mx - 82) : abs(my - 82);
      if (ring != 1)
        fill_module(q, mx, my, QUIRC_PIXEL_BLACK);
    }

  int centre = 30 + 82 * PITCH + PITCH / 2;
  grid->align.x = centre + off_x * PITCH;
  grid->align.y = centre + off_y * PITCH;
  find_alignment_pattern(q, 0);

  int found_x = grid->align.x;
  int found_y = grid->align.y;
  if (grid->align_region >= 0) {
    found_x = q->regions[grid->align_region].seed.x;
    found_y = q->regions[grid->align_region].seed.y;
  }
  if (abs(found_x - centre) > PITCH / 2 || abs(found_y - centre) > PITCH / 2) {
    printf("FAIL pattern %+d,%+d modules from the estimate: (%d,%d), not "
           "(%d,%d)\n",
           off_x, off_y, found_x, found_y, centre, centre);
    failures++;
  } else {
    printf("ok   pattern %+d,%+d modules from the estimate\n", off_x, off_y);
  }
  k_quirc_destroy(q);
}

int main(void) {
  check_left_alone("estimate far outside the image", 8678, 250901, 7 * PITCH,
                   7 * PITCH);
  check_left_alone("estimate left of the image", -40, 300, 7 * PITCH,
                   7 * PITCH);
  check_left_alone("finders without a size", 300, 300, 0, 0);
  check_left_alone("finders larger than the image", 300, 300, 100000, 100000);
  check_left_alone("pitch that extrapolates past infinity", 300, 300, 42, 14);
  check_left_alone("nothing resembling the pattern in reach", 300, 300,
                   7 * PITCH, 7 * PITCH);

  check_found(0, 0);
  check_found(3, -2);
  check_found(-3, 3);
  check_found(-4, -4);

  if (failures) {
    printf("%d alignment search test(s) failed\n", failures);
    return 1;
  }
  printf("All alignment search tests passed.\n");
  return 0;
}
