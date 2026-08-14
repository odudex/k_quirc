/*
 * Version-database invariant test.
 *
 * The version DB is pure transcribed ISO/IEC 18004 data, so it cannot be
 * validated by decoding images alone: a wrong entry simply makes every symbol
 * of that version/ECC combination fail, which is indistinguishable from an
 * optics limit. This test checks the table directly, both for internal
 * consistency and against an independent capacity oracle.
 *
 * It exists because v25/Q shipped with ns=3 instead of ns=7, which silently
 * broke every version-25 Q-level symbol and was masked in the validation
 * suite as 18 "known failures" attributed to pixels-per-module.
 */

#include <stdio.h>

#include "k_quirc.h"
#include "k_quirc_internal.h"

static int failures;

static void fail(int version, const char *level, const char *fmt, int a,
                 int b) {
  fprintf(stderr, "v%d/%s: ", version, level);
  fprintf(stderr, fmt, a, b);
  fprintf(stderr, "\n");
  failures++;
}

/* ISO/IEC 18004 total codewords (data + error correction) per version. */
static const int iso_total_codewords[] = {
    0,    26,   44,   70,   100,  134,  172,  196, 242, 292,
    346,  404,  466,  532,  581,  655,  733,  815, 901, 991,
    1085, 1156, 1258, 1364, 1474, 1588, 1706, 1828};

/* ISO/IEC 18004 total DATA codewords per version, in L, M, Q, H order.
 * This is the independent oracle: it is not derived from bs/dw/ns, so a
 * self-consistent but incorrect block description still fails here. */
static const int iso_data_codewords[][4] = {
    {0, 0, 0, 0},          {19, 16, 13, 9},        {34, 28, 22, 16},
    {55, 44, 34, 26},      {80, 64, 48, 36},       {108, 86, 62, 46},
    {136, 108, 76, 60},    {156, 124, 88, 66},     {194, 154, 110, 86},
    {232, 182, 132, 100},  {274, 216, 154, 122},   {324, 254, 180, 140},
    {370, 290, 206, 158},  {428, 334, 244, 180},   {461, 365, 261, 197},
    {523, 415, 295, 223},  {589, 453, 325, 253},   {647, 507, 367, 283},
    {721, 563, 397, 313},  {795, 627, 445, 341},   {861, 669, 485, 385},
    {932, 714, 512, 406},  {1006, 782, 568, 442},  {1094, 860, 614, 464},
    {1174, 914, 664, 514}, {1276, 1000, 718, 538}, {1370, 1062, 754, 596},
    {1468, 1128, 808, 628}};

/* quirc_version_info.ecc[] is indexed by the K_QUIRC_ECC_LEVEL_* constants,
 * which are M, L, H, Q -- not the L, M, Q, H order used by the spec tables. */
static const int ecc_index[4] = {K_QUIRC_ECC_LEVEL_L, K_QUIRC_ECC_LEVEL_M,
                                 K_QUIRC_ECC_LEVEL_Q, K_QUIRC_ECC_LEVEL_H};
static const char *ecc_name[4] = {"L", "M", "Q", "H"};

int main(void) {
  const int max_ver = (int)(sizeof(iso_total_codewords) / sizeof(int)) - 1;

  if (max_ver != QUIRC_MAX_VERSION) {
    fprintf(stderr,
            "oracle covers versions 1-%d but QUIRC_MAX_VERSION is %d -- "
            "extend iso_total_codewords[] and iso_data_codewords[]\n",
            max_ver, QUIRC_MAX_VERSION);
    return 1;
  }

  for (int v = 1; v <= QUIRC_MAX_VERSION; v++) {
    const struct quirc_version_info *info = &quirc_version_db[v];
    const int grid = 4 * v + 17;

    if (info->data_bytes != iso_total_codewords[v])
      fail(v, "-", "data_bytes %d != ISO total %d", info->data_bytes,
           iso_total_codewords[v]);

    /* Buffers must be able to hold the largest supported symbol. */
    if ((grid * grid + 7) / 8 > K_QUIRC_MAX_BITMAP)
      fail(v, "-", "bitmap %d exceeds K_QUIRC_MAX_BITMAP %d",
           (grid * grid + 7) / 8, K_QUIRC_MAX_BITMAP);
    if (info->data_bytes > K_QUIRC_MAX_PAYLOAD)
      fail(v, "-", "data_bytes %d exceeds K_QUIRC_MAX_PAYLOAD %d",
           info->data_bytes, K_QUIRC_MAX_PAYLOAD);

    /* Alignment pattern centres: start at 6, strictly increasing, terminated
     * inside the array, and inside the symbol. */
    if (v > 1) {
      int prev = 0;
      for (int i = 0; i < QUIRC_MAX_ALIGNMENT && info->apat[i]; i++) {
        if (i == 0 && info->apat[i] != 6)
          fail(v, "-", "apat[0] is %d, expected 6", info->apat[i], 0);
        if (info->apat[i] <= prev && i > 0)
          fail(v, "-", "apat not increasing at %d (%d)", i, info->apat[i]);
        if (info->apat[i] > grid - 7)
          fail(v, "-", "apat %d outside grid %d", info->apat[i], grid);
        prev = info->apat[i];
      }
    }

    for (int e = 0; e < 4; e++) {
      const struct quirc_rs_params *p = &info->ecc[ecc_index[e]];
      const char *name = ecc_name[e];
      const int bs = p->bs, dw = p->dw, ns = p->ns;

      if (ns < 1 || dw < 1 || dw >= bs) {
        fail(v, name, "implausible bs/dw (%d/%d)", bs, dw);
        continue;
      }

      /* The decoder computes lb_count by integer division at
       * k_quirc_decode.c:474. That is only meaningful if the division is
       * exact -- an inexact remainder means the block description does not
       * account for every codeword, and every block then de-interleaves at
       * the wrong stride. This is the check that catches the v25/Q bug. */
      const int rem = info->data_bytes - bs * ns;
      if (rem < 0 || rem % (bs + 1) != 0) {
        fail(v, name,
             "inexact partition: data_bytes - bs*ns = %d, not a "
             "multiple of bs+1 = %d",
             rem, bs + 1);
        continue;
      }

      const int lb_count = rem / (bs + 1);
      const int bc = ns + lb_count;

      if (bs * ns + (bs + 1) * lb_count != info->data_bytes)
        fail(v, name, "block sizes sum to %d, expected %d",
             bs * ns + (bs + 1) * lb_count, info->data_bytes);
      if (bc > 255)
        fail(v, name, "block count %d exceeds %d", bc, 255);

      /* Independent oracle: total data codewords must match the spec. */
      const int data_cw = dw * ns + (dw + 1) * lb_count;
      if (data_cw != iso_data_codewords[v][e])
        fail(v, name, "data codewords %d != ISO %d", data_cw,
             iso_data_codewords[v][e]);

      /* ecc_offset (k_quirc_decode.c:477) must stay inside the codewords. */
      if (dw * bc + lb_count > info->data_bytes)
        fail(v, name, "ecc_offset %d exceeds data_bytes %d", dw * bc + lb_count,
             info->data_bytes);
    }
  }

  if (failures) {
    fprintf(stderr, "version DB: %d failure(s)\n", failures);
    return 1;
  }
  printf("version DB: versions 1-%d x 4 ECC levels OK\n", QUIRC_MAX_VERSION);
  return 0;
}
