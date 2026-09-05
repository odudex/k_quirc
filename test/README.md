# k_quirc Desktop Test

Standalone test harness that runs the k_quirc QR decoder on grayscale images outside of ESP-IDF. Validates the full detection and decoding pipeline on desktop Linux and reports per-image results with timing.

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

The test reads all `.pgm` and `.png` files from `test/pgm_samples/`. This directory is not checked in — you need to supply your own images of QR codes.

Supported formats:

- **PGM** — P5 (binary) grayscale
- **PNG** — any color depth (automatically converted to grayscale via `stb_image`)

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

## Capacity and ROI Transitions

`k_quirc_resize_254_test` and `k_quirc_resize_1024_test` cover shared 8-bit
image/label storage and separate 16-bit label storage. They verify that resizing
within capacity makes no image allocations, invalid sizes and failed growth
preserve the context, shrinking clears discarded pixels, and growth/destruction
scrub the full allocation. Repeated full-frame/ROI decoding of a checked-in QR
must match a fresh decoder's results.

Run all self-contained regressions with `ctest --test-dir build --output-on-failure`
after building. No external samples are needed. From the Kern repository root,
`just test` builds and runs them alongside the other host tests; configure with
`-DK_QUIRC_SANITIZE=ON` for ASan/UBSan.

## Two-Level Images

`k_quirc_bimodal_test` covers strictly bimodal input — a QR rendered on a screen or straight out of a generator, with exactly two grey levels and nothing in between. It quantizes a checked-in vector onto `{51, light}` and sweeps `light`, requiring a decode at every level:

```bash
./build/k_quirc_bimodal_test [image]
```

The gap between the two levels makes Otsu's between-class variance flat across a whole range of thresholds, so which end of that plateau the argmax reports decides whether the adaptive offset lands in the gap or on top of a mode. The pgm samples and the validation matrix cannot see this: their light level is a pure 255, where the offset clamps and the failure hides.

## Malformed Payloads

`k_quirc_decode_payload_test` drives `decode_payload()` with hand-built bitstreams instead of images, covering symbols that pass Reed-Solomon but carry segment data no real encoder would emit:

```bash
./build/k_quirc_decode_payload_test
```

- **Alphanumeric bounds** — the 6-bit tail field holds 0..63 and the 11-bit pair field 0..2047, but the character table has only 45 entries. Values 45/63 and 2025/2047 are asserted to be rejected, 44 and 2024 accepted.
- **Segment-mode mask** — `data_type` is the OR of every mode present, so `[Kanji][byte]` must report both. Callers reject Kanji by testing that field, and an assignment-per-segment left only the trailing mode visible.

Everything the generator produces is well-formed by construction, so the validation matrix cannot reach either case. The test `#include`s `k_quirc_decode.c` to reach the static segment decoders, which is why that file is not also linked into the target.

## Sanitizers

Configure a second build directory to run the whole harness under ASan and UBSan:

```bash
cmake -B build_asan -DK_QUIRC_SANITIZE=ON
cmake --build build_asan
./build_asan/k_quirc_decode_payload_test
```

Keep it separate from the plain build — the validation suite runs an order of magnitude slower under sanitizers. UBSan is the half that matters most here: both defects the malformed-payload test covers were invisible to pass/fail (an out-of-bounds read into adjacent `.rodata`, and a left-shift of a negative value), and only a sanitizer reports them.

## Automated Validation

The `validation/` directory contains a separate test suite that generates synthetic QR images across a matrix of versions, ECC levels, encoding modes, and scales, then runs them through this harness and validates that every decoded payload matches the expected data. See [`validation/README.md`](validation/README.md) for details.

A GitHub Actions workflow at `.github/workflows/validate.yml` runs the full validation on push and PR, plus a narrow sanitized sweep over the same image path.
