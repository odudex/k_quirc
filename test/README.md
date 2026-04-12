# k_quirc Desktop Test

Standalone test harness that runs the k_quirc QR decoder on grayscale PGM images outside of ESP-IDF. Validates the full detection and decoding pipeline on desktop Linux and reports per-image results with timing.

## Building

```bash
cd test
cmake -B build
cmake --build build
```

This compiles the core k_quirc sources directly (not as an ESP component) with two optional algorithms enabled:

- `K_QUIRC_ADAPTIVE_THRESHOLD` — auto-adjust binarization threshold from timing patterns
- `K_QUIRC_BILINEAR_THRESHOLD` — four-quadrant Otsu thresholding for uneven lighting

## Test Images

The test reads all `.pgm` files from `test/pgm_samples/`. This directory is not checked in — you need to supply your own P5 (binary) PGM images of QR codes.

To convert other formats:

```bash
convert input.png -colorspace Gray output.pgm
```

## Running

```bash
./build/k_quirc_test
```

### Output

A table with one row per image:

```
Sample                        Decoded  Time(ms)  Caps   Grids   Error               Payload
------------------------------------------------------------------------------------------------------------
sample_01.pgm                 YES         4.21  3      1                             https://example.com/...
sample_02.pgm                 NO          3.87  3      1       format error          gs=25 v2 align=YES caps=[0,1,2]
```

| Column  | Meaning |
|---------|---------|
| Caps    | Finder patterns (capstones) detected |
| Grids   | QR grids formed from capstone grouping |
| Error   | `no finder patterns`, `no grid formed`, or a decode error from `k_quirc_strerror` |
| Payload | First 40 characters of decoded data |

### Failure Diagnostics

When a sample fails to decode, the harness automatically:

- **Dumps capstone geometry** if 3+ capstones were found but no grid formed — prints perspective-unmap coordinates and alignment ratios to diagnose grouping failures.
- **Sweeps threshold offsets** from -20 to +20 (step 5) and reports which offsets would have succeeded, indicating threshold sensitivity.

## Stubs

The `stubs/` directory provides minimal stand-ins for ESP-IDF and FreeRTOS headers so the k_quirc sources compile on desktop:

| Stub | What it does |
|------|-------------|
| `esp_log.h` | Routes `ESP_LOGW`/`ESP_LOGE` to stderr, silences info/debug |
| `freertos/FreeRTOS.h` | Empty header |
| `freertos/task.h` | `vTaskDelay` becomes a no-op |
