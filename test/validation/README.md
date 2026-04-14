# QR Decode Validation Suite

Automated regression testing for the k_quirc QR decoder. Generates synthetic QR test images, runs them through the desktop test harness, and verifies that every decoded payload matches the expected data byte-for-byte.

## What This Validates

The core question: given a QR code with a known payload, does k_quirc decode it correctly and return the exact expected data?

Each test image places a QR code at a specific scale onto a gray background within a frame of a given resolution. These are clean, synthetic images — no noise, rotation, or perspective distortion — designed to isolate decoder correctness from image quality concerns. The harness must locate the QR code, binarize the image, detect finder patterns, form a grid, correct perspective, extract data, and apply error correction. A payload mismatch at any stage means the test fails.

### What we're protecting against

- **Decode regressions** — code changes that break decoding for specific parameter combinations. The alignment centroid bug ([PR #2](https://github.com/odudex/k_quirc/pull/2)) is the motivating example: it broke v2-v3 decoding at high pixels-per-module values and went undetected because there was no automated validation.
- **Encoding mode coverage** — the decoder must correctly handle numeric, alphanumeric, and byte-mode QR codes. Different modes use different bit-packing and data layout within the QR grid, exercising different decoder code paths.
- **Scale-dependent failures** — the decoder must work across a range of image scales (pixels-per-module) and frame sizes. Bugs can manifest at specific scales where pixel boundaries align poorly with QR module boundaries.
- **ECC level sensitivity** — different error correction levels change the data/ECC ratio within the grid, altering the internal structure the decoder must parse correctly.

## How It Works

### Image Generation

QR images are generated using [`qrencode`](https://github.com/fukuchi/libqrencode) (a C-based CLI tool), then composited onto a larger gray background using Pillow:

1. `qrencode` renders the QR code as a PNG at the specified version, ECC level, and pixels-per-module
2. The script loads the PNG, pastes it centered on a mid-gray (128) background at the target frame size, and saves the result

The gray surround ensures the QR code doesn't fill the entire image, so the decoder must locate it within the frame — as it would when processing a real camera frame. The images are otherwise ideal: perfect contrast, no noise, axis-aligned.

### Test Matrix

The test matrix is parameterized across five dimensions:

| Dimension | Default | Description |
|-----------|---------|-------------|
| Version | 1-25 | QR symbol version (grid size) |
| ECC | L, M, Q, H | Error correction level |
| Mode | numeric, alphanumeric, byte | Encoding mode (determines payload character set) |
| Scale | PPM 3,5,8,13,20 + 90% fill | Pixels-per-module (fixed values and fill-percentage) |
| Frame | 320, 480, 640 | Image dimensions in pixels |

Each configuration runs 3 iterations with different random payloads. Combinations where the QR image would exceed the frame are skipped at matrix-build time.

**Two rendering modes** produce complementary coverage:

- **Fixed PPM** — every QR version is rendered at the same set of pixels-per-module values (e.g., 3, 5, 8, 13, 20). A v1 QR code at PPM=8 produces a small image, while a v25 at PPM=8 produces a large one. This tests whether the decoder handles a given scale correctly regardless of QR complexity. Combinations where the resulting image exceeds the frame are skipped (high versions at high PPM don't fit in small frames).

- **Fill percentage** — models the common real-world scenario where a user holds a QR code up to fill the viewfinder. The PPM is computed so that the QR code occupies a target percentage of the frame (default 90%). This naturally affects small and large QR codes differently: a v1 code (21 modules) filling a 640px frame gets a high PPM with large, easy-to-resolve modules, while a v25 code (133 modules) filling a 320px frame is forced to a very low PPM where each module is only a few pixels wide. The result is a different computed PPM for every version/frame combination, covering scale points that the fixed PPM list might miss.

### Payload Generation

Payloads are filled with deterministic random data seeded from the test parameters via `hashlib.md5`. This ensures results are reproducible across runs while maximizing character variety. Payload lengths vary randomly between 10% and 100% of capacity.

- **Numeric**: random digits (`0-9`)
- **Alphanumeric**: random characters from the 45-char QR alphanumeric set
- **Byte**: random bytes across the full `0x00-0xFF` range, including nulls

Payload length is capped at the decoder's buffer limit (`K_QUIRC_MAX_PAYLOAD = 2560` bytes) to avoid false "Data overflow" failures that would be test-script artifacts rather than real decode issues.

### Validation

The test harness outputs hex-encoded payloads (via the `K_QUIRC_HEX_OUTPUT` environment variable). The script regenerates each expected payload from the same deterministic seed and compares hex representations. A mismatch or decode failure is a test failure.

### Known Failures

[`known_failures.txt`](known_failures.txt) lists filename base patterns for test cases that fail due to decoder limitations at boundary conditions (not regressions). Each line is a pattern like `v01_L_numeric_ppm03_320x320` that matches all iterations (`_r0.png`, `_r1.png`, etc.).

The validation classifies every result into one of four categories:

| Status | Meaning |
|--------|---------|
| `passed` | Decoded correctly, as expected |
| `expected_failure` | Failed, but listed in `known_failures.txt` — not a regression |
| `unexpected_failure` | Failed and NOT in known failures — **this is a regression** |
| `newly_passing` | Listed in known failures but now passes — **notable improvement** |

**Exit code**: 0 if no regressions (expected failures and newly-passing cases are OK). 1 if any unexpected failures.

When a decoder improvement causes known failures to start passing, the report flags them. Remove them from `known_failures.txt` once confirmed stable.

### Current Known Failures

Two clusters of boundary-condition failures:

- **v1 at PPM=3** — 3 pixels per module is at the resolution floor for the finder pattern detector
- **v25 at low PPM** — large QR grids (133 modules including quiet zone) crammed into small frames at 2-5 pixels per module

## Usage

### Prerequisites

```
apt install qrencode        # C-based QR code generator
pip install Pillow          # image compositing
cmake, gcc or clang         # to build the test harness
```

### Quick Start

```bash
# From the k_quirc repo root:
cd test && cmake -B build && cmake --build build && cd ..

python3 test/validation/run_validation.py --binary test/build/k_quirc_test
```

### CLI Options

```
--versions RANGE       QR versions, e.g. "1-25" or "5" (default: 1-25)
--ecc LEVELS           Comma-separated ECC levels (default: L,M,Q,H)
--modes MODES          Comma-separated modes (default: numeric,alphanumeric,byte)
--ppm VALUES           Comma-separated PPM values (default: 3,5,8,13,20)
--fill PERCENTAGES     Comma-separated fill percentages (default: 90)
--frames SIZES         Comma-separated frame sizes (default: 320,480,640)
--iterations N         Random payloads per configuration (default: 3)
--json                 Machine-parseable JSON output to stdout
--binary PATH          Path to pre-built k_quirc_test (skips build step)
--build-dir NAME       Build directory name (default: build_ci)
--csv-path PATH        CSV output path (default: validation-results.csv)
--failures-dir DIR     Directory for regression images (default: test/validation/failures/)
--known-failures PATH  Path to known failures file (default: test/validation/known_failures.txt)
--k-quirc-dir DIR      k_quirc repo root (default: auto-detect from script location)
```

### Examples

```bash
# Minimal single-case smoke test
python3 test/validation/run_validation.py --versions 1 --ecc L --modes numeric \
    --ppm 5 --frames 320 --iterations 1 --binary test/build/k_quirc_test

# Byte mode edge cases at high ECC
python3 test/validation/run_validation.py --versions 1-5 --ecc H --modes byte \
    --binary test/build/k_quirc_test

# Full matrix with JSON output
python3 test/validation/run_validation.py --json --binary test/build/k_quirc_test \
    > results.json
```

### Output

**Human-readable report** (stderr):
```
Validation Results
============================================================
Total test cases:    8244
Passed:              8160
Expected failures:   82
Newly passing:       2  *** NOTABLE ***
Skipped:             2652
Effective pass rate: 100.0%
Total decode time:   7987.1 ms
```

**CSV** (one row per test case): version, ecc, mode, render type, ppm, frame, filename, decoded, payload_valid, status, timing, capstones, grids, error.

**JSON** (`--json`): machine-parseable summary with full regression/improvement/skip details.

## CI Integration

The GitHub Actions workflow at `.github/workflows/validate.yml` runs the full default matrix on every push to `master` and on pull requests. Manual dispatch exposes all parameters for targeted investigation.

The full matrix (~8,200 test cases) runs in under a minute.

## File Layout

```
test/validation/
  run_validation.py      # test scenario generation and validation script
  known_failures.txt     # expected boundary-condition failures
  requirements.txt       # Python dependencies (Pillow)
  README.md              # this file
```
