# Adaptive Threshold Convergence Test

Tests the *dynamics* of the `K_QUIRC_ADAPTIVE_THRESHOLD` loop, which nothing
else in the suite can observe. The loop measures frame N and corrects frame
N+1, but `k_quirc_test` and the validation matrix run one frame per image — so
the correction is discarded with the context — and their vectors are sharp, and
a sharp QR decodes at every offset from -60 to +90.

This test drives degraded vectors through a **single long-lived context**,
repeating each frame so that the only thing moving between iterations is the
controller, from four start offsets. It does **not** try several offsets within
one frame: the live scanner cannot afford the decode time, so the loop gets one
attempt per frame and that is what is measured.

The set is built so that **no fixed offset can pass it**. Eight vectors need a
progressively higher offset (up to +35) while the two heavy-ink vectors stop
decoding above −10 and +0, so the best any constant achieves is 8/10.

## Metrics

| Column | Meaning |
|--------|---------|
| Lock | First decoding frame, worst case over start offsets; `-` if some start never decoded. On an animated QR these are payload parts missed, each costing a full cycle to come round again. |
| Yield | Decode rate over the final frames — the fraction of parts captured per cycle once settled. |
| Settled | Final offset. A range means different starts ended elsewhere, so the loop had not converged. |
| Hunt | Runs whose offset reversed direction after settling. Reversals re-threshold every frame differently and drop parts indefinitely, so this must stay 0. |
| Moving | Runs still changing on the last frame. |

## Running

```bash
cd test && cmake -B build && cmake --build build
./build/k_quirc_convergence_test
./build/k_quirc_convergence_test --verbose                      # trajectories
./build/k_quirc_convergence_test --min-lock 10 --min-yield 100  # enforce
./build/k_quirc_convergence_test --dir ../pgm_samples           # real captures
```

Without `--min-lock`/`--min-yield` it reports and exits 0. CI should pass
explicit thresholds so regressions fail the build.

## Baseline

Finder-area controller versus the timing-pattern one it replaced:

| | vectors: old | current | real captures: old | current |
|---|---|---|---|---|
| Locked | 5/10 | **10/10** | 24/64 | **26/64** |
| Yield | 55% | **100%** | 38% | **43%** |
| Hunting | 0/40 | 0/40 | 2/256 | **1/256** |
| Not converged | 5/40 | **0/40** | 20/256 | **1/256** |
| Frames to lock | 28-29 | **0-4** | | |

Yield on the real set is capped well below 100% because only about 40% of those
captures decode at *any* offset — most are lost to resolution or focus, not
thresholding.

### Stability

The offset accumulates while the measurement settles within the frame, so the
error decays as `e[n+1] = (1 − L)·e[n]` with `L = GAIN × Kp`. Measured across
the real captures the plant gain `Kp = d(dilation)/d(offset)` spans 0.0005 to
0.0033 — a sevenfold spread:

| plant | L | pole | settling to 95% |
|---|---|---|---|
| slowest measured | 0.09 | +0.91 | 32 frames |
| median | 0.36 | +0.64 | 6.7 frames |
| fastest measured | 0.66 | +0.34 | 2.8 frames |

The pole stays real and positive throughout, so the approach is monotone from
any starting offset. Overshoot needs `L > 1` (1.5× the fastest measured plant),
instability `L ≥ 2` (3×). The gain buys that margin rather than the shortest
settling time, which is why `Hunt` is 0 rather than merely small.

## Regenerating the vectors

Vectors are committed so the C test needs no Python at build time. Edit `CASES`
in `gen_vectors.py` and re-run it; needs `qrencode`, Pillow and numpy.

`degrade.py` applies a physically ordered camera model — the subject is printed
before it is viewed, projected before it is blurred, and blurred before the
sensor responds:

```
scene -> linearize -> ink spread -> perspective -> defocus -> motion blur
      -> illumination -> exposure gain -> highlight clip -> gamma encode
      -> flare/range -> sensor noise
```

**Geometry and optics act on linear light.** Blurring or resampling the
gamma-encoded image instead — the obvious shortcut — inverts the sign of the
distortion: black *grows* into white (δ = +0.17) where a real camera erodes it
(δ = −0.14), so vectors built that way would tune the threshold backwards.

Defocus is in **pixels**, not modules, because real defocus is fixed by the
lens: its effect grows as pixels-per-module shrinks. The projective resample
carries its own point spread, so tilted vectors need less explicit defocus.

Parameters are calibrated to reproduce the finder-area signature of the real
captures — outer ring, white ring and centre stone as fractions of the finder,
with δ the implied black dilation in modules:

| | ring | white | stone | δ |
|---|---|---|---|---|
| ideal / sharp control | 0.490 | 0.327 | 0.184 | +0.000 |
| real camera captures | 0.379 | 0.445 | 0.177 | −0.142 |
| defocused + tilted | 0.358 | 0.456 | 0.179 | −0.156 |
| heavy ink | 0.618 | 0.133 | 0.250 | +0.256 |

The defocused vectors sit slightly harsher than reality, which is deliberate
headroom. The heavy-ink pair are the only ones on the far side of ideal, and
they are what stops a one-directional controller from passing. The sensor noise
matters too: without it the vectors were too clean to provoke the limit cycling
real captures cause, and the test scored a hunting controller as perfect.
