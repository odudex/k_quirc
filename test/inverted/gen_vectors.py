#!/usr/bin/env python3
"""Generate the inverted (light-on-dark) QR test vectors.

The vectors are committed to git, so this only needs re-running when the set
changes.  Requires qrencode, Pillow and numpy.

    python3 gen_vectors.py [output_dir]

Each vector is a QR rendered with qrencode and negated - light modules on a
dark ground, quiet zone included - then composited onto a frame and passed
through the camera model in ../convergence/degrade.py.  That is what a camera
sees of a code shown by a dark-themed screen, or engraved into dark metal.

The payload follows from the version alone, so a test can rebuild it from the
file name: the one the convergence vectors carry, cut short below version 4,
where that one does not fit.
"""
import os
import shutil
import subprocess
import sys
import warnings

warnings.simplefilter('ignore')

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'convergence'))
from degrade import degrade

QUIET_ZONE_MODULES = 4
ECC = 'M'
FRAME_SIZE = 480   # the resolution Krux actually captures at

# Per-vector scene and distortion:
#   surround    gray level of the scene around the code's dark quiet zone: a
#               dark screen's own background, or the lighter page a dark label
#               sits on
#   lo, hi      the levels black and white reach the sensor at.  A dim screen
#               is nowhere near 0 and 255, and a threshold that assumes the
#               two levels straddle mid-gray reads nothing from it
#   sigma       optical defocus, in pixels
#   ev          exposure gain.  A camera metering a mostly dark frame
#               overexposes the ink, which then blooms into the ground
#   gradient    linear illumination ramp
#   rx, ry, rz  viewing-geometry tilt in degrees
def _case(version, ppm, surround=32, lo=0, hi=255, sigma=0.0, ev=0.0,
          gradient=0.0, rx=0.0, ry=0.0, rz=0.0, noise=1.5):
    return dict(version=version, ppm=ppm, surround=surround, lo=lo, hi=hi,
                sigma=sigma, ev=ev, gradient=gradient, rx=rx, ry=ry, rz=rz,
                noise=noise)


CASES = [
    _case(1, 10, surround=0, noise=0.0),                 # two levels, 0/255
    _case(6,  8, lo=30, hi=110, noise=0.0),              # dim screen
    _case(6,  8, surround=160, lo=140, hi=235, noise=0.0),  # washed-out print
    _case(6,  6, sigma=0.9, rx=10, ry=-6, rz=3),
    _case(10, 6, sigma=0.9, ev=0.5, rx=14, ry=8, rz=2),
    _case(10, 5, sigma=1.2, ev=0.5, rx=-6, ry=-14, rz=5),
    _case(10, 6, sigma=0.9, gradient=0.25, rx=-12, ry=7, rz=-2),
    _case(14, 5, surround=128, sigma=0.6, rx=9, ry=-11, rz=4),
]


def payload_for(version):
    repeats = version * 2 if version >= 4 else version
    return "V%02d:" % version + "ABCDEFGHIJ" * repeats


def render(version, ppm, surround, path):
    payload = payload_for(version)
    total_modules = (17 + 4 * version) + 2 * QUIET_ZONE_MODULES
    if total_modules * ppm > FRAME_SIZE:
        return None
    result = subprocess.run(
        ['qrencode', '-v', str(version), '-l', ECC, '-s', str(ppm),
         '-m', str(QUIET_ZONE_MODULES), '-t', 'PNG', '-o', path],
        input=payload.encode(), capture_output=True)
    if result.returncode != 0:
        return None
    qr = Image.open(path).convert('L').point(lambda v: 255 - v)
    if qr.width != total_modules * ppm:
        return None     # qrencode needed a larger version for the payload
    frame = Image.new('L', (FRAME_SIZE, FRAME_SIZE), surround)
    frame.paste(qr, ((FRAME_SIZE - qr.width) // 2,
                     (FRAME_SIZE - qr.height) // 2))
    return np.asarray(frame)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'vectors')
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)

    for c in CASES:
        name = ("inv_v%02d_ppm%02d_sig%03d_ev%02d_gr%02d_tilt%02d"
                "_bg%03d_lv%03d-%03d.png"
                % (c['version'], c['ppm'], int(c['sigma'] * 100),
                   int(c['ev'] * 10), int(c['gradient'] * 100),
                   int(round((c['rx'] ** 2 + c['ry'] ** 2) ** 0.5)),
                   c['surround'], c['lo'], c['hi']))
        path = os.path.join(out, name)
        pixels = render(c['version'], c['ppm'], c['surround'], path)
        if pixels is None:
            sys.exit("does not fit, or qrencode failed: %s" % name)
        Image.fromarray(degrade(pixels, rx=c['rx'], ry=c['ry'], rz=c['rz'],
                                sigma_px=c['sigma'], ev=c['ev'],
                                gradient=c['gradient'], gradient_angle=30.0,
                                black_level=c['lo'], white_ceiling=c['hi'],
                                noise=c['noise'], seed=7)).save(path)
        print("  %s" % name)


if __name__ == '__main__':
    main()
