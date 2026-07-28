/*
 * Adaptive-threshold control loop convergence test.  See convergence/README.md
 * for the metrics and the recorded baseline.
 *
 * The controller is closed over a video stream: it measures frame N and
 * corrects frame N+1, which the decode-correctness suites cannot observe since
 * they run one frame per image.  Feeding the SAME degraded frame to a single
 * long-lived context makes the plant stationary, so the only thing changing
 * between iterations is the controller itself.
 */

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_LINEAR
#define STBI_NO_HDR
#include "stb_image.h"

#include "k_quirc.h"
#include "k_quirc_internal.h"

#define MAX_VECTORS 64
#define FRAMES 40
#define STEADY_FRAMES 15

/* Convergence must not depend on where the loop happens to start. */
static const int start_offsets[] = {-20, 0, 10, 20};
#define NUM_STARTS ((int)(sizeof(start_offsets) / sizeof(start_offsets[0])))

static uint8_t *load_pgm(const char *path, int *w, int *h) {
  FILE *f = fopen(path, "rb");
  if (!f)
    return NULL;

  char magic[3];
  if (fscanf(f, "%2s", magic) != 1 || strcmp(magic, "P5") != 0) {
    fclose(f);
    return NULL;
  }

  int c;
  while ((c = fgetc(f)) != EOF) {
    if (c == '#') {
      while ((c = fgetc(f)) != EOF && c != '\n')
        ;
    } else if (c > ' ') {
      ungetc(c, f);
      break;
    }
  }

  int maxval;
  if (fscanf(f, "%d %d %d", w, h, &maxval) != 3) {
    fclose(f);
    return NULL;
  }
  fgetc(f);

  size_t n = (size_t)(*w) * (size_t)(*h);
  uint8_t *data = malloc(n);
  if (!data) {
    fclose(f);
    return NULL;
  }
  if (fread(data, 1, n, f) != n) {
    free(data);
    fclose(f);
    return NULL;
  }
  fclose(f);
  return data;
}

static uint8_t *load_image(const char *path, int *w, int *h) {
  size_t len = strlen(path);
  if (len > 4 && strcmp(path + len - 4, ".png") == 0) {
    int channels;
    return stbi_load(path, w, h, &channels, 1);
  }
  return load_pgm(path, w, h);
}

struct run_result {
  int lock;      /* first decoding frame, or -1 */
  int hits;      /* decodes within the steady window */
  int reversals; /* direction changes in the steady window = hunting */
  int moving;    /* offset still changing on the last frame = not converged */
  int off_final;
};

/* Drive one vector through FRAMES iterations of a single context. */
static struct run_result run_vector(const uint8_t *image, int w, int h,
                                    int start_offset, int verbose) {
  struct run_result r = {-1, 0, 0, 0, start_offset};
  int prev_offset = start_offset;
  int prev_dir = 0;

  k_quirc_t *q = k_quirc_new();
  if (!q)
    return r;
  if (k_quirc_resize(q, w, h) < 0) {
    k_quirc_destroy(q);
    return r;
  }
  k_quirc_set_threshold_offset_for(q, start_offset);

  for (int frame = 0; frame < FRAMES; frame++) {
    /* A fresh frame arrives from the camera every iteration.  The copy is
     * required, not incidental: identification binarizes and flood-fills the
     * buffer in place, so the grayscale does not survive a pass. */
    uint8_t *buf = k_quirc_begin(q, NULL, NULL);
    if (!buf)
      break;
    memcpy(buf, image, (size_t)w * (size_t)h);
    k_quirc_end(q, false);

    int decoded = 0;
    k_quirc_result_t result;
    int count = k_quirc_count(q);
    for (int i = 0; i < count; i++) {
      if (k_quirc_decode(q, i, &result) == K_QUIRC_SUCCESS) {
        decoded = 1;
        break;
      }
    }

    if (decoded && r.lock < 0)
      r.lock = frame;

    int offset = k_quirc_get_threshold_offset_for(q);
    r.off_final = offset;

    int dir = (offset > prev_offset) - (offset < prev_offset);
    if (frame >= FRAMES - STEADY_FRAMES) {
      r.hits += decoded;
      /* A ramp that has not finished is "moving"; a loop that changes its
       * mind about which way to go is "hunting".  Only the second one is a
       * stability defect -- but for animated QR both cost payload parts. */
      if (dir != 0 && prev_dir != 0 && dir != prev_dir)
        r.reversals++;
      if (frame == FRAMES - 1)
        r.moving = (dir != 0);
    }
    if (dir != 0)
      prev_dir = dir;
    prev_offset = offset;

    if (verbose)
      printf(" %+d%s", offset, decoded ? "*" : "");
  }
  if (verbose)
    printf("\n");

  k_quirc_destroy(q);
  return r;
}

static int collect_vectors(const char *dir, char paths[][512]) {
  DIR *d = opendir(dir);
  if (!d) {
    fprintf(stderr, "error: cannot open vector directory '%s'\n", dir);
    return 0;
  }

  int n = 0;
  struct dirent *e;
  while ((e = readdir(d)) != NULL && n < MAX_VECTORS) {
    size_t len = strlen(e->d_name);
    if (len < 5)
      continue;
    if (strcmp(e->d_name + len - 4, ".pgm") != 0 &&
        strcmp(e->d_name + len - 4, ".png") != 0)
      continue;
    snprintf(paths[n], 512, "%s/%s", dir, e->d_name);
    n++;
  }
  closedir(d);

  /* Deterministic ordering so output diffs cleanly between runs. */
  for (int i = 1; i < n; i++) {
    char tmp[512];
    strcpy(tmp, paths[i]);
    int j = i - 1;
    while (j >= 0 && strcmp(paths[j], tmp) > 0) {
      strcpy(paths[j + 1], paths[j]);
      j--;
    }
    strcpy(paths[j + 1], tmp);
  }
  return n;
}

static void usage(const char *argv0) {
  printf("usage: %s [options]\n"
         "  --dir PATH        vector directory (default: built-in)\n"
         "  --min-lock N      require at least N vectors to lock (default 0)\n"
         "  --min-yield PCT   require mean steady yield >= PCT (default 0)\n"
         "  --verbose         print the offset trajectory of every run\n",
         argv0);
}

int main(int argc, char **argv) {
  const char *dir = CONVERGENCE_VECTORS_DIR;
  int min_lock = 0;
  int min_yield = 0;
  int verbose = 0;

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--dir") && i + 1 < argc)
      dir = argv[++i];
    else if (!strcmp(argv[i], "--min-lock") && i + 1 < argc)
      min_lock = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--min-yield") && i + 1 < argc)
      min_yield = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--verbose"))
      verbose = 1;
    else {
      usage(argv[0]);
      return 2;
    }
  }

  static char paths[MAX_VECTORS][512];
  int num = collect_vectors(dir, paths);
  if (num == 0) {
    fprintf(stderr, "error: no vectors found in '%s'\n", dir);
    return 2;
  }

#ifndef K_QUIRC_ADAPTIVE_THRESHOLD
  printf("K_QUIRC_ADAPTIVE_THRESHOLD is disabled: the offset is fixed and\n"
         "this test only reports the static decode rate.\n\n");
#endif

  printf("Control loop convergence: %d frames per run, %d start offsets ",
         FRAMES, NUM_STARTS);
  printf("(");
  for (int s = 0; s < NUM_STARTS; s++)
    printf("%s%+d", s ? ", " : "", start_offsets[s]);
  printf(")\n");
  printf("Steady-state window: last %d frames\n\n", STEADY_FRAMES);

  printf("%-42s %6s %7s %10s %5s %7s\n", "Vector", "Lock", "Yield", "Settled",
         "Hunt", "Moving");
  printf("%-42s %6s %7s %10s %5s %7s\n", "------", "----", "-----", "-------",
         "----", "------");

  int locked_vectors = 0;
  long total_hits = 0;
  long total_slots = 0;
  int total_hunting = 0;
  int total_moving = 0;

  for (int p = 0; p < num; p++) {
    int w, h;
    uint8_t *image = load_image(paths[p], &w, &h);
    if (!image) {
      fprintf(stderr, "warning: cannot load '%s'\n", paths[p]);
      continue;
    }

    const char *base = strrchr(paths[p], '/');
    base = base ? base + 1 : paths[p];

    int worst_lock = -1;
    int all_locked = 1;
    int hits = 0;
    int hunting = 0;
    int moving = 0;
    int settled_lo = 1 << 20, settled_hi = -(1 << 20);

    for (int s = 0; s < NUM_STARTS; s++) {
      if (verbose)
        printf("  %s from %+d:", base, start_offsets[s]);
      struct run_result r = run_vector(image, w, h, start_offsets[s], verbose);
      if (r.lock < 0)
        all_locked = 0;
      else if (r.lock > worst_lock)
        worst_lock = r.lock;
      hits += r.hits;
      if (r.reversals)
        hunting++;
      if (r.moving)
        moving++;
      if (r.off_final < settled_lo)
        settled_lo = r.off_final;
      if (r.off_final > settled_hi)
        settled_hi = r.off_final;
    }

    int slots = NUM_STARTS * STEADY_FRAMES;
    total_hits += hits;
    total_slots += slots;
    total_hunting += hunting;
    total_moving += moving;

    char lock_str[16];
    if (all_locked)
      snprintf(lock_str, sizeof(lock_str), "%d", worst_lock);
    else
      snprintf(lock_str, sizeof(lock_str), "-");

    char settled[16];
    if (settled_lo == settled_hi)
      snprintf(settled, sizeof(settled), "%+d", settled_lo);
    else
      snprintf(settled, sizeof(settled), "%+d..%+d", settled_lo, settled_hi);

    printf("%-42s %6s %6.0f%% %10s %5d %7d\n", base, lock_str,
           100.0 * hits / slots, settled, hunting, moving);

    if (all_locked)
      locked_vectors++;
    free(image);
  }

  double mean_yield = total_slots ? 100.0 * total_hits / total_slots : 0.0;
  printf("\n  Locked from every start offset: %d/%d vectors\n", locked_vectors,
         num);
  printf("  Mean steady-state yield:        %.0f%%\n", mean_yield);
  printf("  Runs still hunting:             %d/%d\n", total_hunting,
         num * NUM_STARTS);
  printf("  Runs not converged after %2d f:  %d/%d\n", FRAMES, total_moving,
         num * NUM_STARTS);

  int failed = 0;
  if (locked_vectors < min_lock) {
    printf("\nFAIL: %d vectors locked, required %d\n", locked_vectors,
           min_lock);
    failed = 1;
  }
  if (mean_yield + 1e-9 < min_yield) {
    printf("\nFAIL: mean yield %.0f%%, required %d%%\n", mean_yield, min_yield);
    failed = 1;
  }
  if (!failed && (min_lock || min_yield))
    printf("\nPASS\n");

  return failed;
}
