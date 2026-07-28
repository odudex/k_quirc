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
./build/k_quirc_convergence_test --min-lock 5 --min-yield 55    # enforce
./build/k_quirc_convergence_test --dir ../pgm_samples           # real captures
```

Without `--min-lock`/`--min-yield` it reports and exits 0. CI should pass
explicit thresholds so regressions fail the build.

## Baseline

The controller as it stands (timing-pattern bias, ±1 step per frame, offset
clamped to ±20):

```
Locked from every start offset: 5/10 vectors
Mean steady-state yield:        55%
Runs still hunting:             0/40
Runs not converged after 40 f:  5/40
```

It is stable and correctly signed: it never hunts, does not drift off the sharp
control vector, and walks *down* on the heavy-ink vectors. But it is slow —
28-29 frames to lock, 3-4 s at 10-15 fps — and range-limited: five vectors ramp
into the ±20 clamp and stop there, still undecoded.

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
they are what stops a one-directional controller from passing. Sensor noise
matters too: without it the vectors are too clean to provoke the limit cycling
real captures cause.
