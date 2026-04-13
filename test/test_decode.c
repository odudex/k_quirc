#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_NO_LINEAR
#define STBI_NO_HDR
#include "stb_image.h"

#include "k_quirc.h"
#include "k_quirc_internal.h"

/* Import perspective_unmap for diagnostics */
static void diag_perspective_unmap(const float *c, const struct quirc_point *in,
                                   float *u, float *v) {
  float x = in->x;
  float y = in->y;
  float den = -c[0] * c[7] * y + c[1] * c[6] * y +
              (c[3] * c[7] - c[4] * c[6]) * x + c[0] * c[4] - c[1] * c[3];
  *u = -(c[1] * (y - c[5]) - c[2] * c[7] * y + (c[5] * c[7] - c[4]) * x +
         c[2] * c[4]) /
       den;
  *v = (c[0] * (y - c[5]) - c[2] * c[6] * y + (c[5] * c[6] - c[3]) * x +
        c[2] * c[3]) /
       den;
}

static void dump_grouping_diag(const struct k_quirc *q) {
  printf("    Capstone centers:");
  for (int i = 0; i < q->num_capstones; i++)
    printf(" [%d]=(%d,%d)", i, q->capstones[i].center.x,
           q->capstones[i].center.y);
  printf("\n");

  for (int i = 0; i < q->num_capstones; i++) {
    const struct quirc_capstone *c1 = &q->capstones[i];
    for (int j = 0; j < q->num_capstones; j++) {
      if (i == j)
        continue;
      const struct quirc_capstone *c2 = &q->capstones[j];
      float u, v;
      diag_perspective_unmap(c1->c, &c2->center, &u, &v);
      float du = fabsf(u - 3.5f);
      float dv = fabsf(v - 3.5f);
      const char *h_ok = (du < 0.2f * dv) ? "H" : " ";
      const char *v_ok = (dv < 0.2f * du) ? "V" : " ";
      float sq = (dv > 0.001f) ? du / dv : 999.0f;
      printf("    cap[%d]->cap[%d]: u=%.2f v=%.2f du=%.2f dv=%.2f "
             "ratio=%.3f %s%s\n",
             i, j, u, v, du, dv, sq, h_ok, v_ok);
    }
  }
}

#define MAX_RESULTS 4
#define MAX_FILES 64
#define PAYLOAD_DISPLAY_LEN 40

static uint8_t *load_pgm(const char *path, int *w, int *h) {
  FILE *f = fopen(path, "rb");
  if (!f)
    return NULL;

  char magic[3];
  if (fscanf(f, "%2s", magic) != 1 || strcmp(magic, "P5") != 0) {
    fclose(f);
    return NULL;
  }

  /* Skip whitespace and comment lines */
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

  int width, height, maxval;
  if (fscanf(f, "%d %d", &width, &height) != 2) {
    fclose(f);
    return NULL;
  }
  if (fscanf(f, "%d", &maxval) != 1 || maxval > 255) {
    fclose(f);
    return NULL;
  }
  /* Consume the single whitespace byte after maxval */
  fgetc(f);

  size_t size = (size_t)width * height;
  uint8_t *pixels = malloc(size);
  if (!pixels) {
    fclose(f);
    return NULL;
  }

  if (fread(pixels, 1, size, f) != size) {
    free(pixels);
    fclose(f);
    return NULL;
  }

  fclose(f);
  *w = width;
  *h = height;
  return pixels;
}

static double elapsed_ms(struct timespec *t0, struct timespec *t1) {
  double sec = (double)(t1->tv_sec - t0->tv_sec);
  double nsec = (double)(t1->tv_nsec - t0->tv_nsec);
  return sec * 1000.0 + nsec / 1e6;
}

static int cmp_strings(const void *a, const void *b) {
  return strcmp(*(const char **)a, *(const char **)b);
}

static int ends_with(const char *s, const char *suffix) {
  size_t slen = strlen(s);
  size_t suflen = strlen(suffix);
  if (slen < suflen)
    return 0;
  return strcmp(s + slen - suflen, suffix) == 0;
}

static uint8_t *load_png(const char *path, int *w, int *h) {
  int channels;
  uint8_t *pixels = stbi_load(path, w, h, &channels, 1); /* force grayscale */
  return pixels;
}

static uint8_t *load_image(const char *path, int *w, int *h) {
  if (ends_with(path, ".pgm"))
    return load_pgm(path, w, h);
  if (ends_with(path, ".png"))
    return load_png(path, w, h);
  return NULL;
}

/* Try to decode a sample with a given threshold offset.
 * Returns 1 if decoded, 0 otherwise. */
static int try_decode(k_quirc_t *q, const uint8_t *pixels, int w, int h,
                      int thresh_offset, k_quirc_result_t *result,
                      k_quirc_error_t *out_err, int *out_caps, int *out_grids) {
  k_quirc_set_threshold_offset(thresh_offset);

  uint8_t *buf = k_quirc_begin(q, NULL, NULL);
  memcpy(buf, pixels, w * h);
  k_quirc_end(q, false);

  *out_grids = k_quirc_count(q);
  *out_caps = q->num_capstones;
  *out_err = K_QUIRC_SUCCESS;

  for (int j = 0; j < *out_grids; j++) {
    *out_err = k_quirc_decode(q, j, result);
    if (*out_err == K_QUIRC_SUCCESS && result->valid)
      return 1;
  }
  return 0;
}

int main(void) {
  DIR *dir = opendir(SAMPLES_DIR);
  if (!dir) {
    fprintf(stderr, "Cannot open samples directory: %s\n", SAMPLES_DIR);
    return 1;
  }

  char *filenames[MAX_FILES];
  int nfiles = 0;
  struct dirent *ent;
  while ((ent = readdir(dir)) != NULL && nfiles < MAX_FILES) {
    if (ends_with(ent->d_name, ".pgm") || ends_with(ent->d_name, ".png")) {
      filenames[nfiles++] = strdup(ent->d_name);
    }
  }
  closedir(dir);

  qsort(filenames, nfiles, sizeof(char *), cmp_strings);

  printf("%-28s  %-7s  %8s  %-6s %-6s  %-18s  %s\n", "Sample", "Decoded",
         "Time(ms)", "Caps", "Grids", "Error", "Payload");
  printf("--------------------------------------------------------------"
         "----------------------------------------------\n");

  int total = 0, decoded_count = 0;
  double total_time = 0;

  /* Threshold offsets to sweep when default fails */
  static const int sweep_offsets[] = {-20, -15, -10, -5, 0, 5, 10, 15, 20};
  static const int nsweep = sizeof(sweep_offsets) / sizeof(sweep_offsets[0]);

  k_quirc_t *q = k_quirc_new();

  for (int i = 0; i < nfiles; i++) {
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", SAMPLES_DIR, filenames[i]);

    int w, h;
    uint8_t *pixels = load_image(path, &w, &h);
    if (!pixels) {
      printf("%-28s  %-7s  %8s  %-6s %-6s  %-18s  %s\n", filenames[i], "ERR",
             "-", "-", "-", "load failed", "");
      free(filenames[i]);
      continue;
    }

    if (k_quirc_resize(q, w, h) < 0) {
      printf("%-28s  %-7s  %8s  %-6s %-6s  %-18s  %s\n", filenames[i], "ERR",
             "-", "-", "-", "resize failed", "");
      free(pixels);
      free(filenames[i]);
      continue;
    }

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    /* First try with default offset (10) */
    k_quirc_result_t result;
    k_quirc_error_t err;
    int caps, grids;
    int decoded = try_decode(q, pixels, w, h, 10, &result, &err, &caps, &grids);

    clock_gettime(CLOCK_MONOTONIC, &t1);
    double ms = elapsed_ms(&t0, &t1);
    total_time += ms;
    total++;

    /* Get grid size for diagnostics */
    int grid_size = 0;
    if (grids > 0)
      grid_size = q->grids[0].grid_size;

    char caps_str[8], grids_str[8];
    snprintf(caps_str, sizeof(caps_str), "%d", caps);
    snprintf(grids_str, sizeof(grids_str), "%d", grids);

    if (decoded) {
      decoded_count++;
      char payload_preview[PAYLOAD_DISPLAY_LEN + 4];
      int len = result.data.payload_len;
      if (len > PAYLOAD_DISPLAY_LEN) {
        memcpy(payload_preview, result.data.payload, PAYLOAD_DISPLAY_LEN);
        strcpy(payload_preview + PAYLOAD_DISPLAY_LEN, "...");
      } else {
        memcpy(payload_preview, result.data.payload, len);
        payload_preview[len] = '\0';
      }
      printf("%-28s  %-7s  %8.2f  %-6s %-6s  %-18s  %s\n", filenames[i], "YES",
             ms, caps_str, grids_str, "", payload_preview);
    } else {
      const char *err_str = "";
      if (grids == 0 && caps == 0)
        err_str = "no finder patterns";
      else if (grids == 0)
        err_str = "no grid formed";
      else
        err_str = k_quirc_strerror(err);

      printf("%-28s  %-7s  %8.2f  %-6s %-6s  %-18s", filenames[i], "NO", ms,
             caps_str, grids_str, err_str);
      if (grid_size > 0) {
        int align_found = (q->grids[0].align_region >= 0);
        struct quirc_grid *gr = &q->grids[0];
        printf("  gs=%d v%d align=%s caps=[%d,%d,%d]", grid_size,
               (grid_size - 17) / 4, align_found ? "YES" : "NO", gr->caps[0],
               gr->caps[1], gr->caps[2]);
      }
      printf("\n");

      if (grids == 0 && q->num_capstones >= 3)
        dump_grouping_diag(q);

      /* Show capstone positions for ECC failures with extra caps */
      if (grids > 0 && caps > 3) {
        printf("    All capstone centers:");
        for (int c = 0; c < caps; c++)
          printf(" [%d]=(%d,%d)", c, q->capstones[c].center.x,
                 q->capstones[c].center.y);
        printf("\n");
      }

      /* Sweep threshold offsets to see if any work */
      int sweep_hits = 0;
      char sweep_buf[128] = "";
      int sweep_pos = 0;
      for (int s = 0; s < nsweep; s++) {
        k_quirc_result_t sr;
        k_quirc_error_t se;
        int sc, sg;
        if (try_decode(q, pixels, w, h, sweep_offsets[s], &sr, &se, &sc, &sg)) {
          sweep_pos +=
              snprintf(sweep_buf + sweep_pos, sizeof(sweep_buf) - sweep_pos,
                       "%s%d", sweep_hits ? "," : "", sweep_offsets[s]);
          sweep_hits++;
        }
      }
      if (sweep_hits)
        printf("    --> Would decode at offset(s): %s\n", sweep_buf);
    }

    free(pixels);
    free(filenames[i]);
  }

  k_quirc_destroy(q);

  printf("--------------------------------------------------------------"
         "----------------------------------------------\n");
  printf("Summary: %d/%d decoded (%.0f%%), total: %.1f ms, avg: %.1f ms\n",
         decoded_count, total, total > 0 ? 100.0 * decoded_count / total : 0.0,
         total_time, total > 0 ? total_time / total : 0.0);

  return 0;
}
