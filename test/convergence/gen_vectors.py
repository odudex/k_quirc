#!/usr/bin/env python3
"""Generate the convergence-test vector set.

The vectors are committed to git, so this only needs re-running when the set
changes.  Requires qrencode, Pillow and numpy.

    python3 gen_vectors.py [output_dir]

Each vector is a QR rendered with qrencode, composited onto a gray frame, then
passed through the camera degradation model in degrade.py.  The degradation
parameters are calibrated so the vectors reproduce the capstone-area signature
measured on real Krux camera captures:

    outer ring / white ring / centre stone area fractions
        ideal (undistorted)     0.490 / 0.327 / 0.184
        real camera captures    0.379 / 0.445 / 0.177
        sigma 1.5-2.0, +0.5 EV  0.393 / 0.428 / 0.179

Sharp vectors are useless for this test: they decode at every threshold offset
from -60 to +90, so they cannot observe the control loop at all.  Defocus is
what makes the threshold offset matter.
"""
import os
import shutil
import subprocess
import sys
import warnings

warnings.simplefilter('ignore')

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from degrade import degrade

QUIET_ZONE_MODULES = 4
MID_GRAY = 128
ECC = 'M'
FRAME_SIZE = 480   # the resolution Krux actually captures at

# Per-vector distortion:
#   sigma       optical defocus, in PIXELS (fixed by the lens, so its effect
#               grows as pixels-per-module shrinks)
#   ev          exposure gain; positive overexposes and erodes black
#   ink         printing ink/toner spread, in pixels; fattens black modules
#   gradient    linear illumination ramp, exercises bilinear thresholding
#   rx, ry, rz  viewing-geometry tilt in degrees.  Handheld capture is never
#               fronto-parallel; a tilted QR also means the finder patterns
#               sit at different depths, so a single global threshold offset
#               is a compromise across them.
#
# Defocus is lower here than a fronto-parallel vector would need: the
# projective resample carries its own point spread, and on its own already
# accounts for about a third of the black erosion these vectors exhibit.
#
# The last two vectors pull the threshold the OPPOSITE way from all the
# others.  Without them a controller that can only ever raise its offset
# would score full marks.
def _case(version, ppm, sigma=0.0, ev=0.0, ink=0.0, gradient=0.0,
          rx=0.0, ry=0.0, rz=0.0, noise=1.5):
    return dict(version=version, ppm=ppm, sigma=sigma, ev=ev, ink=ink,
                gradient=gradient, rx=rx, ry=ry, rz=rz, noise=noise)


CASES = [
    _case(6,  8, noise=0.0),                                   # sharp control
    _case(6,  8, sigma=0.9, ev=0.5, rx=10, ry=-6, rz=3),
    _case(6,  6, sigma=0.9, ev=0.5, rx=-8, ry=12, rz=-4),
    _case(10, 6, sigma=0.9, ev=0.5, rx=14, ry=8, rz=2),
    _case(10, 5, sigma=1.2, ev=0.5, rx=-6, ry=-14, rz=5),
    _case(10, 6, sigma=1.2, ev=1.0, rx=12, ry=10, rz=-3),
    _case(14, 5, sigma=0.9, ev=0.5, rx=9, ry=-11, rz=4),
    _case(10, 6, sigma=0.9, ev=0.5, gradient=0.25, rx=-12, ry=7, rz=-2),
    _case(10, 5, sigma=0.6, ink=2.0, rx=8, ry=-10, rz=3),      # heavy ink
    _case(10, 6, sigma=0.6, ink=2.6, rx=-9, ry=8, rz=-3),      # heavy ink
]


def render(version, ppm, path):
    payload = "V%02d:" % version + "ABCDEFGHIJ" * (version * 2)
    total_modules = (17 + 4 * version) + 2 * QUIET_ZONE_MODULES
    if total_modules * ppm > FRAME_SIZE:
        return None
    result = subprocess.run(
        ['qrencode', '-v', str(version), '-l', ECC, '-s', str(ppm),
         '-m', str(QUIET_ZONE_MODULES), '-t', 'PNG', '-o', path],
        input=payload.encode(), capture_output=True)
    if result.returncode != 0:
        return None
    qr = Image.open(path).convert('L')
    frame = Image.new('L', (FRAME_SIZE, FRAME_SIZE), MID_GRAY)
    frame.paste(qr, ((FRAME_SIZE - qr.width) // 2,
                     (FRAME_SIZE - qr.height) // 2))
    frame.save(path)
    return 100.0 * qr.width / FRAME_SIZE


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vectors')
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)

    for c in CASES:
        name = ("cv_v%02d_ppm%02d_sig%03d_ev%02d_ink%02d_gr%02d_tilt%02d.png"
                % (c['version'], c['ppm'], int(c['sigma'] * 100),
                   int(c['ev'] * 10), int(c['ink'] * 10),
                   int(c['gradient'] * 100),
                   int(round((c['rx'] ** 2 + c['ry'] ** 2) ** 0.5))))
        path = os.path.join(out, name)
        fill = render(c['version'], c['ppm'], path)
        if fill is None:
            print("skipped (does not fit / qrencode failed): %s" % name,
                  file=sys.stderr)
            continue
        pixels = np.asarray(Image.open(path).convert('L'))
        Image.fromarray(degrade(pixels, ink=c['ink'],
                                rx=c['rx'], ry=c['ry'], rz=c['rz'],
                                sigma_px=c['sigma'], ev=c['ev'],
                                gradient=c['gradient'], gradient_angle=30.0,
                                noise=c['noise'], seed=7)).save(path)
        print("  %-54s fill %.0f%%  tilt (%+d,%+d,%+d)" % (
            name, fill, c['rx'], c['ry'], c['rz']))


if __name__ == '__main__':
    main()
