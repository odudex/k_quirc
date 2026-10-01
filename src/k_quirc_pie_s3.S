/*
 * K-Quirc kernels for the ESP32-S3's PIE vector unit
 */

    .section .rodata
    .align 4
bias:
    .byte 0x80
one:
    .byte 1

    .text
    .align 4

/* void k_quirc_binarize_pie(uint8_t *pixels, int blocks, const uint8_t *t,
 *                           int t_stride)
 *
 * pixels[i] = pixels[i] < *t, over `blocks` runs of 16 pixels, t advancing
 * t_stride bytes a run.  The S3 compares signed bytes only, so both sides are
 * biased by 0x80 first.  The pixels must be 16-byte aligned. */
    .global k_quirc_binarize_pie
    .type k_quirc_binarize_pie, @function
k_quirc_binarize_pie:
    entry a1, 16
    movi a6, bias
    ee.vldbc.8 q3, a6
    movi a6, one
    ee.vldbc.8 q2, a6
    loopnez a3, 2f
    ee.vldbc.8.xp q1, a4, a5
    ee.vld.128.ip q0, a2, 0
    ee.xorq q1, q1, q3
    ee.xorq q0, q0, q3
    ee.vcmp.lt.s8 q0, q0, q1
    ee.andq q0, q0, q2
    ee.vst.128.ip q0, a2, 16
2:
    retw.n
    .size k_quirc_binarize_pie, .-k_quirc_binarize_pie
