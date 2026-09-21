/*
 * K-Quirc kernels for the ESP32-P4's PIE vector unit
 */

    .section .rodata
one:
    .byte 1

    .text
    .align 4

/* void k_quirc_binarize_pie(uint8_t *pixels, int blocks, const uint8_t *t,
 *                           int t_stride)
 *
 * pixels[i] = pixels[i] < *t, over `blocks` runs of 16 pixels, t advancing
 * t_stride bytes a run: 0 holds one threshold throughout, 1 takes one per run.
 * The pixels must be 16-byte aligned: the vector load and store ignore the low
 * bits of the address. */
    .global k_quirc_binarize_pie
    .type k_quirc_binarize_pie, @function
k_quirc_binarize_pie:
    la a4, one
    esp.vldbc.8.ip q2, a4, 0
1:
    esp.vldbc.8.xp q1, a2, a3
    esp.vld.128.ip q0, a0, 0
    esp.vcmp.lt.u8 q0, q0, q1
    esp.andq       q0, q0, q2
    esp.vst.128.ip q0, a0, 16
    addi a1, a1, -1
    bnez a1, 1b
    ret
    .size k_quirc_binarize_pie, .-k_quirc_binarize_pie
