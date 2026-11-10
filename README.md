# K-Quirc

A standalone QR code recognition and decoding library optimized for embedded systems. Adapted from [OpenMV](https://github.com/openmv/openmv)'s quirc implementation, itself based on Daniel Beer's [quirc](https://github.com/dlbeer/quirc).

## Features

- QR code versions 1-25 (up to ~2,500-byte payloads)
- All four ECC levels (L/M/Q/H)
- Numeric, Alphanumeric, Byte, and Kanji data types
- Inverted (light-on-dark) QR codes, when built with `K_QUIRC_INVERTED_RETRY` (off by default)
- Multiple QR codes per image
- Extended Channel Interpretation (ECI) support

### Optimizations over upstream quirc

- **White-level thresholding** -- Otsu's threshold of the centre, scaled by the white level measured per block and interpolated between blocks, handles uneven lighting
- **Adaptive threshold** -- auto-adjusts offset from timing pattern analysis for improved reliability
- **Span-based flood fill** -- line-based fill with explicit stack avoids recursion; neighbor-row scans resume where they stopped instead of rescanning from the span edge
- **Constant-threshold binarization spans** -- the interpolated row threshold is decomposed into exact constant spans, so the per-pixel loop is branch-free and SIMD-friendly
- **Word-at-a-time pixel scanning** -- finder-pattern runs and flood-fill spans are scanned 4 bytes per step
- **Single-division grid fitness** -- the 3x3 perspective samples per cell share one reciprocal refined by Newton-Raphson, avoiding 8 of 9 FPU divisions
- **ESP32 memory layout** -- large buffers in SPIRAM (cache-aligned), temporary buffers in internal RAM
- **FreeRTOS integration** -- task yields during long loops to prevent watchdog timeout

On non-ESP platforms, memory allocation falls back to standard `malloc`/`free`.

## Usage

### ESP-IDF component

Add as a component in your ESP-IDF project and include the header:

```c
#include "k_quirc.h"
```

### One-shot API

For simple use cases, decode grayscale image data in a single call:

```c
k_quirc_result_t results[4];
int count = k_quirc_decode_grayscale(grayscale_buf, width, height,
                                     results, 4, false);
for (int i = 0; i < count; i++) {
    if (results[i].valid) {
        printf("QR: %.*s\n", results[i].data.payload_len,
               results[i].data.payload);
    }
}
```

### Step-by-step API

For repeated scanning (e.g. camera feed), reuse the decoder instance:

```c
// Create once
k_quirc_t *q = k_quirc_new();
k_quirc_resize(q, width, height);

// Per frame
int w, h;
uint8_t *buf = k_quirc_begin(q, &w, &h);
// Fill buf with grayscale image data (w * h bytes)
memcpy(buf, frame_data, w * h);
k_quirc_end(q, false);  // true to also read inverted QR codes (needs K_QUIRC_INVERTED_RETRY)

int count = k_quirc_count(q);
for (int i = 0; i < count; i++) {
    k_quirc_result_t result;
    k_quirc_error_t err = k_quirc_decode(q, i, &result);
    if (err == K_QUIRC_SUCCESS && result.valid) {
        // result.data.payload, result.data.payload_len
        // result.corners[0..3] for position
    }
}

// Cleanup
k_quirc_destroy(q);
```

## Build options

Feature flags are set as compile definitions in `CMakeLists.txt`:

| Flag | Default | Description |
|------|---------|-------------|
| `K_QUIRC_BILINEAR_THRESHOLD` | Enabled | Threshold follows the local white level, for uneven lighting |
| `K_QUIRC_ADAPTIVE_THRESHOLD` | Enabled | Auto-adjust threshold offset from timing patterns |
| `K_QUIRC_INVERTED_RETRY` | Disabled | Read inverted (light-on-dark) codes: when a frame shows no code and the caller passed `find_inverted`, its negative is searched too |
| `K_QUIRC_DEBUG` | Disabled | Debug visualization (thresholded buffer with grid overlays) |
| `K_QUIRC_THRESHOLD_GRID` | `0` | Blocks per side the white level is measured over; `0` derives it from the image (one per 64 pixels), `1`-`8` fixes it for a project scanning at one resolution |

### Inverted codes

Inverted codes are not read by default. The `find_inverted` argument of `k_quirc_end()` and `k_quirc_decode_grayscale()` only takes effect in a build that defines `K_QUIRC_INVERTED_RETRY`; without it the argument is accepted and ignored, and a light-on-dark code is simply not found.

With it, a frame in which no dark-on-light code was located is searched a second time as its negative. The second search reuses the first one's binarization, so it needs no extra memory and no second thresholding, but it does repeat the finder scan: a scanner that passes `find_inverted` pays for that on every frame without a code in view. A frame is read as one polarity or the other, never both.

## Constraints

- Maximum image size: 1280x1280 pixels
- Maximum QR version: 25 (117x117 modules)
- Flood-fill stack: 8,192 entries

## License

MIT -- see source file headers for full copyright notices.

**Original:** Daniel Beer (2010-2012)
**OpenMV modifications:** Ibrahim Abdelkader, Kwabena W. Agyeman (2013-2021)
