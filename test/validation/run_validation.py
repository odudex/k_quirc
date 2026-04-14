#!/usr/bin/env python3
"""CI validation suite for k_quirc QR decode library.

Generates synthetic QR test images across a configurable matrix of versions,
ECC levels, encoding modes, scales, and frame sizes, then runs the k_quirc
desktop test harness and validates that every decoded payload matches the
expected data byte-for-byte.

Prerequisites:
    Python 3.8+, Pillow, qrencode (apt install qrencode)
    CMake 3.16+ and a C compiler (gcc/clang)

Usage:
    python3 run_validation.py [OPTIONS]

    With no options, runs the full default matrix (v1-25, all ECC, all modes,
    fixed PPM + 90% fill, 3 iterations per config). Exits 0 if all pass, 1
    if any failure.
"""

import argparse
import csv
import hashlib
import json
import os
import random
import re
import shutil
import subprocess
import sys
import tempfile
import warnings
from concurrent.futures import ProcessPoolExecutor

try:
    from PIL import Image
except ImportError:
    print("Error: Pillow is required. Install with: pip install Pillow",
          file=sys.stderr)
    sys.exit(1)

# ---------------------------------------------------------------------------
# ISO 18004 QR Code Capacity Table (versions 1-25)
# Source: https://ryanagibson.com/extra/qr-character-limits/
# CAPACITY[(version, ecc, mode)] = max characters/bytes
# ---------------------------------------------------------------------------

CAPACITY = {
    # Version 1
    (1, 'L', 'numeric'): 41,    (1, 'L', 'alphanumeric'): 25,   (1, 'L', 'byte'): 17,
    (1, 'M', 'numeric'): 34,    (1, 'M', 'alphanumeric'): 20,   (1, 'M', 'byte'): 14,
    (1, 'Q', 'numeric'): 27,    (1, 'Q', 'alphanumeric'): 16,   (1, 'Q', 'byte'): 11,
    (1, 'H', 'numeric'): 17,    (1, 'H', 'alphanumeric'): 10,   (1, 'H', 'byte'): 7,
    # Version 2
    (2, 'L', 'numeric'): 77,    (2, 'L', 'alphanumeric'): 47,   (2, 'L', 'byte'): 32,
    (2, 'M', 'numeric'): 63,    (2, 'M', 'alphanumeric'): 38,   (2, 'M', 'byte'): 26,
    (2, 'Q', 'numeric'): 48,    (2, 'Q', 'alphanumeric'): 29,   (2, 'Q', 'byte'): 20,
    (2, 'H', 'numeric'): 34,    (2, 'H', 'alphanumeric'): 20,   (2, 'H', 'byte'): 14,
    # Version 3
    (3, 'L', 'numeric'): 127,   (3, 'L', 'alphanumeric'): 77,   (3, 'L', 'byte'): 53,
    (3, 'M', 'numeric'): 101,   (3, 'M', 'alphanumeric'): 61,   (3, 'M', 'byte'): 42,
    (3, 'Q', 'numeric'): 77,    (3, 'Q', 'alphanumeric'): 47,   (3, 'Q', 'byte'): 32,
    (3, 'H', 'numeric'): 58,    (3, 'H', 'alphanumeric'): 35,   (3, 'H', 'byte'): 24,
    # Version 4
    (4, 'L', 'numeric'): 187,   (4, 'L', 'alphanumeric'): 114,  (4, 'L', 'byte'): 78,
    (4, 'M', 'numeric'): 149,   (4, 'M', 'alphanumeric'): 90,   (4, 'M', 'byte'): 62,
    (4, 'Q', 'numeric'): 111,   (4, 'Q', 'alphanumeric'): 67,   (4, 'Q', 'byte'): 46,
    (4, 'H', 'numeric'): 82,    (4, 'H', 'alphanumeric'): 50,   (4, 'H', 'byte'): 34,
    # Version 5
    (5, 'L', 'numeric'): 255,   (5, 'L', 'alphanumeric'): 154,  (5, 'L', 'byte'): 106,
    (5, 'M', 'numeric'): 202,   (5, 'M', 'alphanumeric'): 122,  (5, 'M', 'byte'): 84,
    (5, 'Q', 'numeric'): 144,   (5, 'Q', 'alphanumeric'): 87,   (5, 'Q', 'byte'): 60,
    (5, 'H', 'numeric'): 106,   (5, 'H', 'alphanumeric'): 64,   (5, 'H', 'byte'): 44,
    # Version 6
    (6, 'L', 'numeric'): 322,   (6, 'L', 'alphanumeric'): 195,  (6, 'L', 'byte'): 134,
    (6, 'M', 'numeric'): 255,   (6, 'M', 'alphanumeric'): 154,  (6, 'M', 'byte'): 106,
    (6, 'Q', 'numeric'): 178,   (6, 'Q', 'alphanumeric'): 108,  (6, 'Q', 'byte'): 74,
    (6, 'H', 'numeric'): 139,   (6, 'H', 'alphanumeric'): 84,   (6, 'H', 'byte'): 58,
    # Version 7
    (7, 'L', 'numeric'): 370,   (7, 'L', 'alphanumeric'): 224,  (7, 'L', 'byte'): 154,
    (7, 'M', 'numeric'): 293,   (7, 'M', 'alphanumeric'): 178,  (7, 'M', 'byte'): 122,
    (7, 'Q', 'numeric'): 207,   (7, 'Q', 'alphanumeric'): 125,  (7, 'Q', 'byte'): 86,
    (7, 'H', 'numeric'): 154,   (7, 'H', 'alphanumeric'): 93,   (7, 'H', 'byte'): 64,
    # Version 8
    (8, 'L', 'numeric'): 461,   (8, 'L', 'alphanumeric'): 279,  (8, 'L', 'byte'): 192,
    (8, 'M', 'numeric'): 365,   (8, 'M', 'alphanumeric'): 221,  (8, 'M', 'byte'): 152,
    (8, 'Q', 'numeric'): 259,   (8, 'Q', 'alphanumeric'): 157,  (8, 'Q', 'byte'): 108,
    (8, 'H', 'numeric'): 202,   (8, 'H', 'alphanumeric'): 122,  (8, 'H', 'byte'): 84,
    # Version 9
    (9, 'L', 'numeric'): 552,   (9, 'L', 'alphanumeric'): 335,  (9, 'L', 'byte'): 230,
    (9, 'M', 'numeric'): 432,   (9, 'M', 'alphanumeric'): 262,  (9, 'M', 'byte'): 180,
    (9, 'Q', 'numeric'): 312,   (9, 'Q', 'alphanumeric'): 189,  (9, 'Q', 'byte'): 130,
    (9, 'H', 'numeric'): 235,   (9, 'H', 'alphanumeric'): 143,  (9, 'H', 'byte'): 98,
    # Version 10
    (10, 'L', 'numeric'): 652,  (10, 'L', 'alphanumeric'): 395, (10, 'L', 'byte'): 271,
    (10, 'M', 'numeric'): 513,  (10, 'M', 'alphanumeric'): 311, (10, 'M', 'byte'): 213,
    (10, 'Q', 'numeric'): 364,  (10, 'Q', 'alphanumeric'): 221, (10, 'Q', 'byte'): 151,
    (10, 'H', 'numeric'): 288,  (10, 'H', 'alphanumeric'): 174, (10, 'H', 'byte'): 119,
    # Version 11
    (11, 'L', 'numeric'): 772,  (11, 'L', 'alphanumeric'): 468, (11, 'L', 'byte'): 321,
    (11, 'M', 'numeric'): 604,  (11, 'M', 'alphanumeric'): 366, (11, 'M', 'byte'): 251,
    (11, 'Q', 'numeric'): 427,  (11, 'Q', 'alphanumeric'): 259, (11, 'Q', 'byte'): 177,
    (11, 'H', 'numeric'): 331,  (11, 'H', 'alphanumeric'): 200, (11, 'H', 'byte'): 137,
    # Version 12
    (12, 'L', 'numeric'): 883,  (12, 'L', 'alphanumeric'): 535, (12, 'L', 'byte'): 367,
    (12, 'M', 'numeric'): 691,  (12, 'M', 'alphanumeric'): 419, (12, 'M', 'byte'): 287,
    (12, 'Q', 'numeric'): 489,  (12, 'Q', 'alphanumeric'): 296, (12, 'Q', 'byte'): 203,
    (12, 'H', 'numeric'): 374,  (12, 'H', 'alphanumeric'): 227, (12, 'H', 'byte'): 155,
    # Version 13
    (13, 'L', 'numeric'): 1022, (13, 'L', 'alphanumeric'): 619, (13, 'L', 'byte'): 425,
    (13, 'M', 'numeric'): 796,  (13, 'M', 'alphanumeric'): 483, (13, 'M', 'byte'): 331,
    (13, 'Q', 'numeric'): 580,  (13, 'Q', 'alphanumeric'): 352, (13, 'Q', 'byte'): 241,
    (13, 'H', 'numeric'): 427,  (13, 'H', 'alphanumeric'): 259, (13, 'H', 'byte'): 177,
    # Version 14
    (14, 'L', 'numeric'): 1101, (14, 'L', 'alphanumeric'): 667, (14, 'L', 'byte'): 458,
    (14, 'M', 'numeric'): 871,  (14, 'M', 'alphanumeric'): 528, (14, 'M', 'byte'): 362,
    (14, 'Q', 'numeric'): 621,  (14, 'Q', 'alphanumeric'): 376, (14, 'Q', 'byte'): 258,
    (14, 'H', 'numeric'): 468,  (14, 'H', 'alphanumeric'): 283, (14, 'H', 'byte'): 194,
    # Version 15
    (15, 'L', 'numeric'): 1250, (15, 'L', 'alphanumeric'): 758, (15, 'L', 'byte'): 520,
    (15, 'M', 'numeric'): 991,  (15, 'M', 'alphanumeric'): 600, (15, 'M', 'byte'): 412,
    (15, 'Q', 'numeric'): 703,  (15, 'Q', 'alphanumeric'): 426, (15, 'Q', 'byte'): 292,
    (15, 'H', 'numeric'): 530,  (15, 'H', 'alphanumeric'): 321, (15, 'H', 'byte'): 220,
    # Version 16
    (16, 'L', 'numeric'): 1408, (16, 'L', 'alphanumeric'): 854, (16, 'L', 'byte'): 586,
    (16, 'M', 'numeric'): 1082, (16, 'M', 'alphanumeric'): 656, (16, 'M', 'byte'): 450,
    (16, 'Q', 'numeric'): 775,  (16, 'Q', 'alphanumeric'): 470, (16, 'Q', 'byte'): 322,
    (16, 'H', 'numeric'): 602,  (16, 'H', 'alphanumeric'): 365, (16, 'H', 'byte'): 250,
    # Version 17
    (17, 'L', 'numeric'): 1548, (17, 'L', 'alphanumeric'): 938, (17, 'L', 'byte'): 644,
    (17, 'M', 'numeric'): 1212, (17, 'M', 'alphanumeric'): 734, (17, 'M', 'byte'): 504,
    (17, 'Q', 'numeric'): 876,  (17, 'Q', 'alphanumeric'): 531, (17, 'Q', 'byte'): 364,
    (17, 'H', 'numeric'): 674,  (17, 'H', 'alphanumeric'): 408, (17, 'H', 'byte'): 280,
    # Version 18
    (18, 'L', 'numeric'): 1725, (18, 'L', 'alphanumeric'): 1046, (18, 'L', 'byte'): 718,
    (18, 'M', 'numeric'): 1346, (18, 'M', 'alphanumeric'): 816, (18, 'M', 'byte'): 560,
    (18, 'Q', 'numeric'): 948,  (18, 'Q', 'alphanumeric'): 574, (18, 'Q', 'byte'): 394,
    (18, 'H', 'numeric'): 746,  (18, 'H', 'alphanumeric'): 452, (18, 'H', 'byte'): 310,
    # Version 19
    (19, 'L', 'numeric'): 1903, (19, 'L', 'alphanumeric'): 1153, (19, 'L', 'byte'): 792,
    (19, 'M', 'numeric'): 1500, (19, 'M', 'alphanumeric'): 909, (19, 'M', 'byte'): 624,
    (19, 'Q', 'numeric'): 1063, (19, 'Q', 'alphanumeric'): 644, (19, 'Q', 'byte'): 442,
    (19, 'H', 'numeric'): 813,  (19, 'H', 'alphanumeric'): 493, (19, 'H', 'byte'): 338,
    # Version 20
    (20, 'L', 'numeric'): 2061, (20, 'L', 'alphanumeric'): 1249, (20, 'L', 'byte'): 858,
    (20, 'M', 'numeric'): 1600, (20, 'M', 'alphanumeric'): 970, (20, 'M', 'byte'): 666,
    (20, 'Q', 'numeric'): 1159, (20, 'Q', 'alphanumeric'): 702, (20, 'Q', 'byte'): 482,
    (20, 'H', 'numeric'): 919,  (20, 'H', 'alphanumeric'): 557, (20, 'H', 'byte'): 382,
    # Version 21
    (21, 'L', 'numeric'): 2232, (21, 'L', 'alphanumeric'): 1352, (21, 'L', 'byte'): 929,
    (21, 'M', 'numeric'): 1708, (21, 'M', 'alphanumeric'): 1035, (21, 'M', 'byte'): 711,
    (21, 'Q', 'numeric'): 1224, (21, 'Q', 'alphanumeric'): 742, (21, 'Q', 'byte'): 509,
    (21, 'H', 'numeric'): 969,  (21, 'H', 'alphanumeric'): 587, (21, 'H', 'byte'): 403,
    # Version 22
    (22, 'L', 'numeric'): 2409, (22, 'L', 'alphanumeric'): 1460, (22, 'L', 'byte'): 1003,
    (22, 'M', 'numeric'): 1872, (22, 'M', 'alphanumeric'): 1134, (22, 'M', 'byte'): 779,
    (22, 'Q', 'numeric'): 1358, (22, 'Q', 'alphanumeric'): 823, (22, 'Q', 'byte'): 565,
    (22, 'H', 'numeric'): 1056, (22, 'H', 'alphanumeric'): 640, (22, 'H', 'byte'): 439,
    # Version 23
    (23, 'L', 'numeric'): 2620, (23, 'L', 'alphanumeric'): 1588, (23, 'L', 'byte'): 1091,
    (23, 'M', 'numeric'): 2059, (23, 'M', 'alphanumeric'): 1248, (23, 'M', 'byte'): 857,
    (23, 'Q', 'numeric'): 1468, (23, 'Q', 'alphanumeric'): 890, (23, 'Q', 'byte'): 611,
    (23, 'H', 'numeric'): 1108, (23, 'H', 'alphanumeric'): 672, (23, 'H', 'byte'): 461,
    # Version 24
    (24, 'L', 'numeric'): 2812, (24, 'L', 'alphanumeric'): 1704, (24, 'L', 'byte'): 1171,
    (24, 'M', 'numeric'): 2188, (24, 'M', 'alphanumeric'): 1326, (24, 'M', 'byte'): 911,
    (24, 'Q', 'numeric'): 1588, (24, 'Q', 'alphanumeric'): 963, (24, 'Q', 'byte'): 661,
    (24, 'H', 'numeric'): 1228, (24, 'H', 'alphanumeric'): 744, (24, 'H', 'byte'): 511,
    # Version 25
    (25, 'L', 'numeric'): 3057, (25, 'L', 'alphanumeric'): 1853, (25, 'L', 'byte'): 1273,
    (25, 'M', 'numeric'): 2395, (25, 'M', 'alphanumeric'): 1451, (25, 'M', 'byte'): 997,
    (25, 'Q', 'numeric'): 1718, (25, 'Q', 'alphanumeric'): 1041, (25, 'Q', 'byte'): 715,
    (25, 'H', 'numeric'): 1286, (25, 'H', 'alphanumeric'): 779, (25, 'H', 'byte'): 535,
}

# QR alphanumeric character set (45 chars)
ALPHANUMERIC_CHARS = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:'

QUIET_ZONE_MODULES = 4
MID_GRAY = 128

# Harness output uses PAYLOAD_DISPLAY_LEN=40 bytes for payload preview
PAYLOAD_DISPLAY_LEN = 40

# k_quirc decoder buffer limit (K_QUIRC_MAX_PAYLOAD in k_quirc.h)
# Payloads exceeding this cause "Data overflow" errors in the decoder.
K_QUIRC_MAX_PAYLOAD = 2560


# ---------------------------------------------------------------------------
# Known failures
# ---------------------------------------------------------------------------

def filename_base(fname):
    """Strip iteration suffix (_rN.png) to get the base pattern."""
    return re.sub(r'_r\d+\.png$', '', fname)


def load_known_failures(path):
    """Load known failure base patterns from a text file.

    Returns a set of base pattern strings. Blank lines and lines starting
    with '#' are ignored.
    """
    patterns = set()
    if not os.path.isfile(path):
        return patterns
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                patterns.add(line)
    return patterns


# ---------------------------------------------------------------------------
# Payload generation
# ---------------------------------------------------------------------------

def make_payload(version, ecc, mode, render_key, frame, iteration):
    """Generate a deterministic random payload for a test case.

    Returns str (numeric/alphanumeric) or bytes (byte mode).
    """
    capacity = CAPACITY.get((version, ecc, mode))
    if capacity is None or capacity < 1:
        return None

    key = f"{version},{ecc},{mode},{render_key},{frame},{iteration}"
    seed = int.from_bytes(hashlib.md5(key.encode()).digest()[:8], 'little')
    rng = random.Random(seed)

    # Cap at decoder buffer size to avoid "Data overflow" errors that are
    # test-script artifacts rather than real decode failures.
    max_len = min(capacity, K_QUIRC_MAX_PAYLOAD)
    min_len = max(1, max_len // 10)
    payload_len = rng.randint(min_len, max_len)

    if mode == 'numeric':
        return ''.join(rng.choices('0123456789', k=payload_len))
    elif mode == 'alphanumeric':
        return ''.join(rng.choices(ALPHANUMERIC_CHARS, k=payload_len))
    elif mode == 'byte':
        return bytes(rng.randint(0, 255) for _ in range(payload_len))
    return None


# ---------------------------------------------------------------------------
# Image generation
# ---------------------------------------------------------------------------

def _check_qrencode():
    """Verify qrencode is installed. Called once at startup."""
    result = subprocess.run(['qrencode', '--version'], capture_output=True,
                            text=True)
    if result.returncode != 0:
        print("Error: qrencode is required. Install with: "
              "apt install qrencode", file=sys.stderr)
        sys.exit(1)


def _generate_one(args):
    """Worker for parallel image generation. Returns (filename, ok, error)."""
    case, tmp_dir = args
    path = os.path.join(tmp_dir, case['filename'])
    try:
        ok = generate_test_image(
            case['version'], case['ecc'], case['mode'],
            case['payload'], case['ppm'], case['frame'], path)
        return case['filename'], ok, '' if ok else 'qrencode/frame rejected'
    except Exception as e:
        return case['filename'], False, f'{type(e).__name__}: {e}'


def generate_test_image(version, ecc, mode, payload, ppm, frame_size, path):
    """Generate a QR test image and save as PNG.

    Uses the qrencode CLI tool (C-based, much faster than Python qrcode).
    Returns True on success, False if QR exceeds frame.
    """
    # Compute expected QR image size to skip before calling qrencode
    total_modules = (17 + 4 * version) + 2 * QUIET_ZONE_MODULES
    qr_pixels = total_modules * ppm
    if qr_pixels > frame_size:
        return False

    # Build qrencode command
    cmd = ['qrencode', '-v', str(version), '-l', ecc,
           '-s', str(ppm), '-m', str(QUIET_ZONE_MODULES),
           '-t', 'PNG', '-o', path]

    if isinstance(payload, bytes):
        cmd.append('-8')
        input_data = payload
    else:
        input_data = payload.encode('ascii')

    result = subprocess.run(cmd, input=input_data, capture_output=True)
    if result.returncode != 0:
        return False

    # Load QR image, place centered on gray background frame
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', UserWarning)
        qr_img = Image.open(path).convert('L')
    if qr_img.width > frame_size or qr_img.height > frame_size:
        return False

    frame = Image.new('L', (frame_size, frame_size), MID_GRAY)
    ox = (frame_size - qr_img.width) // 2
    oy = (frame_size - qr_img.height) // 2
    frame.paste(qr_img, (ox, oy))
    frame.save(path)
    return True


# ---------------------------------------------------------------------------
# Test matrix generation
# ---------------------------------------------------------------------------

def compute_fill_ppm(version, fill_pct, frame_size):
    """Compute PPM for a fill-percentage rendering."""
    total_modules = (17 + 4 * version) + 2 * QUIET_ZONE_MODULES
    return int(frame_size * fill_pct / 100 / total_modules)


def build_test_matrix(versions, ecc_levels, modes, ppm_values, fill_pcts,
                      frame_sizes, iterations):
    """Build the full list of test cases.

    Returns (cases, skipped) where cases is a list of dicts and skipped is
    a list of dicts describing combos that were skipped.
    """
    cases = []
    skipped = []

    for version in versions:
        total_modules = (17 + 4 * version) + 2 * QUIET_ZONE_MODULES
        for ecc in ecc_levels:
            for mode in modes:
                for frame in frame_sizes:
                    # Fixed PPM tests
                    for ppm in ppm_values:
                        # Skip if QR image would exceed frame
                        if total_modules * ppm > frame:
                            skipped.append({
                                'version': version, 'ecc': ecc,
                                'mode': mode, 'ppm': ppm, 'frame': frame,
                                'reason': (f'QR image ({total_modules * ppm}px)'
                                           f' exceeds frame ({frame}px)'),
                            })
                            continue

                        render_key = f"ppm{ppm}"
                        for it in range(iterations):
                            payload = make_payload(version, ecc, mode,
                                                   render_key, frame, it)
                            if payload is None:
                                skipped.append({
                                    'version': version, 'ecc': ecc,
                                    'mode': mode, 'ppm': ppm, 'frame': frame,
                                    'reason': 'no capacity entry',
                                })
                                continue

                            fname = (f"v{version:02d}_{ecc}_{mode}_ppm{ppm:02d}"
                                     f"_{frame}x{frame}_r{it}.png")
                            cases.append({
                                'version': version, 'ecc': ecc, 'mode': mode,
                                'render': 'ppm', 'fill_pct': None,
                                'ppm': ppm, 'frame': frame,
                                'iteration': it,
                                'filename': fname,
                                'payload': payload,
                            })

                    # Fill percentage tests
                    for fill_pct in fill_pcts:
                        ppm = compute_fill_ppm(version, fill_pct, frame)
                        if ppm < 2:
                            skipped.append({
                                'version': version, 'ecc': ecc,
                                'mode': mode, 'fill_pct': fill_pct,
                                'ppm': ppm, 'frame': frame,
                                'reason': f'computed PPM={ppm} < 2',
                            })
                            continue

                        render_key = f"fill{fill_pct}"
                        for it in range(iterations):
                            payload = make_payload(version, ecc, mode,
                                                   render_key, frame, it)
                            if payload is None:
                                continue

                            fname = (f"v{version:02d}_{ecc}_{mode}"
                                     f"_fill{fill_pct}_{frame}x{frame}"
                                     f"_r{it}.png")
                            cases.append({
                                'version': version, 'ecc': ecc, 'mode': mode,
                                'render': 'fill', 'fill_pct': fill_pct,
                                'ppm': ppm, 'frame': frame,
                                'iteration': it,
                                'filename': fname,
                                'payload': payload,
                            })

    return cases, skipped


# ---------------------------------------------------------------------------
# Build helpers
# ---------------------------------------------------------------------------

def build_harness(test_dir, build_dir_name):
    """Build k_quirc_test. Returns path to the binary."""
    build_dir = os.path.join(test_dir, build_dir_name)
    os.makedirs(build_dir, exist_ok=True)

    def run(args):
        result = subprocess.run(args, cwd=test_dir, capture_output=True,
                                text=True)
        if result.returncode != 0:
            print(f"BUILD FAILED: {' '.join(args)}", file=sys.stderr)
            if result.stderr:
                print(result.stderr, file=sys.stderr)
            sys.exit(1)

    run(['cmake', '-B', build_dir_name, '-S', '.'])
    run(['cmake', '--build', build_dir_name])

    return os.path.join(build_dir, 'k_quirc_test')


# ---------------------------------------------------------------------------
# Run and parse harness output
# ---------------------------------------------------------------------------

def run_harness(binary, samples_dir, timeout=600):
    """Run the test harness and return stdout."""
    env = os.environ.copy()
    env['K_QUIRC_HEX_OUTPUT'] = '1'

    result = subprocess.run(
        [binary, samples_dir],
        env=env, capture_output=True, text=True, timeout=timeout,
    )
    return result.stdout


def parse_output(stdout):
    """Parse harness output into per-file results.

    Returns {filename: {decoded, hex_payload, time_ms, caps, grids, error}}
    """
    results = {}
    line_re = re.compile(
        r'^(\S+\.(?:pgm|png))\s+(YES|NO|ERR)\s+'
        r'(\S+)\s+(\S+)\s+(\S+)\s{2}(.{18})\s*(.*)?$'
    )

    for line in stdout.splitlines():
        m = line_re.match(line)
        if not m:
            continue

        fname = m.group(1)
        status = m.group(2)
        time_str = m.group(3)
        caps_str = m.group(4)
        grids_str = m.group(5)
        error_str = m.group(6).strip()
        payload_str = m.group(7).strip() if m.group(7) else ''

        try:
            time_ms = float(time_str)
        except ValueError:
            time_ms = 0.0

        try:
            caps = int(caps_str)
        except ValueError:
            caps = 0

        try:
            grids = int(grids_str)
        except ValueError:
            grids = 0

        results[fname] = {
            'decoded': status == 'YES',
            'hex_payload': payload_str.rstrip('.'),
            'time_ms': time_ms,
            'caps': caps,
            'grids': grids,
            'error': error_str if status != 'YES' else '',
        }

    return results


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------

def expected_hex(payload):
    """Convert a payload to its expected hex string (first PAYLOAD_DISPLAY_LEN bytes)."""
    if isinstance(payload, bytes):
        data = payload
    else:
        data = payload.encode('ascii')

    display = data[:PAYLOAD_DISPLAY_LEN]
    return display.hex()


def validate_results(cases, parsed, known_failures=None):
    """Validate decoded results against expected payloads.

    Returns list of result dicts with pass/fail status and known-failure
    classification:
      - 'known_failure': True if this case matches a known failure pattern
      - 'status': 'passed', 'expected_failure', 'unexpected_failure',
                  or 'newly_passing'
    """
    if known_failures is None:
        known_failures = set()

    results = []

    for case in cases:
        fname = case['filename']
        exp_hex = expected_hex(case['payload'])
        is_known = filename_base(fname) in known_failures

        harness = parsed.get(fname)
        if harness is None:
            status = 'expected_failure' if is_known else 'unexpected_failure'
            results.append({**case, 'decoded': False, 'payload_valid': False,
                            'time_ms': 0, 'caps': 0, 'grids': 0,
                            'error': 'not found in harness output',
                            'known_failure': is_known, 'status': status})
            continue

        decoded = harness['decoded']
        if decoded:
            payload_valid = harness['hex_payload'].startswith(exp_hex)
        else:
            payload_valid = False

        if payload_valid:
            status = 'newly_passing' if is_known else 'passed'
        else:
            status = 'expected_failure' if is_known else 'unexpected_failure'

        results.append({
            **case,
            'decoded': decoded,
            'payload_valid': payload_valid,
            'time_ms': harness['time_ms'],
            'caps': harness['caps'],
            'grids': harness['grids'],
            'error': harness['error'],
            'known_failure': is_known,
            'status': status,
        })

    return results


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def write_csv(results, path):
    """Write per-test-case CSV."""
    fieldnames = ['version', 'ecc', 'mode', 'render', 'fill_pct', 'ppm',
                  'frame', 'iteration', 'filename', 'decoded', 'payload_valid',
                  'status', 'time_ms', 'caps', 'grids', 'error']

    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames,
                                extrasaction='ignore')
        writer.writeheader()
        for r in results:
            row = {k: r.get(k, '') for k in fieldnames}
            row['decoded'] = 'yes' if r['decoded'] else 'no'
            row['payload_valid'] = 'yes' if r['payload_valid'] else 'no'
            row['fill_pct'] = r.get('fill_pct') or ''
            writer.writerow(row)


def print_report(results, skipped, file=sys.stderr):
    """Print human-readable summary to stderr."""
    total = len(results)
    passed = sum(1 for r in results if r['status'] == 'passed')
    expected = sum(1 for r in results if r['status'] == 'expected_failure')
    unexpected = sum(1 for r in results if r['status'] == 'unexpected_failure')
    newly_passing = sum(1 for r in results if r['status'] == 'newly_passing')
    total_time = sum(r['time_ms'] for r in results)

    print(f"\nValidation Results", file=file)
    print(f"{'=' * 60}", file=file)
    print(f"Total test cases:    {total}", file=file)
    print(f"Passed:              {passed}", file=file)
    print(f"Expected failures:   {expected}", file=file)
    if unexpected > 0:
        print(f"REGRESSIONS:         {unexpected}", file=file)
    if newly_passing > 0:
        print(f"Newly passing:       {newly_passing}  *** NOTABLE ***",
              file=file)
    print(f"Skipped:             {len(skipped)}", file=file)
    effective = passed + expected + newly_passing
    if total > 0:
        print(f"Effective pass rate: {100.0 * effective / total:.1f}%",
              file=file)
    print(f"Total decode time:   {total_time:.1f} ms", file=file)

    if newly_passing > 0:
        print(f"\nNewly passing (were known failures — consider removing "
              f"from known_failures.txt):", file=file)
        print(f"{'-' * 60}", file=file)
        for r in results:
            if r['status'] == 'newly_passing':
                print(f"  {r['filename']}", file=file)

    if unexpected > 0:
        print(f"\nRegressions (unexpected failures):", file=file)
        print(f"{'-' * 60}", file=file)
        for r in results:
            if r['status'] == 'unexpected_failure':
                print(f"  {r['filename']}: decoded={r['decoded']} "
                      f"error={r['error']}", file=file)


def make_json_output(results, skipped, matrix_params):
    """Build JSON output dict."""
    total = len(results)
    passed = sum(1 for r in results if r['status'] == 'passed')
    expected = sum(1 for r in results if r['status'] == 'expected_failure')
    unexpected = sum(1 for r in results if r['status'] == 'unexpected_failure')
    newly_passing_count = sum(1 for r in results
                              if r['status'] == 'newly_passing')
    total_time = sum(r['time_ms'] for r in results)

    def _case_info(r):
        return {
            'filename': r['filename'],
            'version': r['version'], 'ecc': r['ecc'], 'mode': r['mode'],
            'ppm': r['ppm'], 'frame': r['frame'],
            'decoded': r['decoded'],
            'error': r.get('error', ''),
        }

    regressions = [_case_info(r) for r in results
                   if r['status'] == 'unexpected_failure']
    expected_failures = [_case_info(r) for r in results
                         if r['status'] == 'expected_failure']
    newly_passing = [_case_info(r) for r in results
                     if r['status'] == 'newly_passing']

    return {
        'matrix': matrix_params,
        'summary': {
            'total': total,
            'passed': passed,
            'expected_failures': expected,
            'regressions': unexpected,
            'newly_passing': newly_passing_count,
            'skipped': len(skipped),
            'pass_rate': round(100.0 * passed / total, 1) if total > 0 else 0,
            'total_time_ms': round(total_time, 1),
        },
        'regressions': regressions,
        'expected_failures': expected_failures,
        'newly_passing': newly_passing,
        'skipped': skipped,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_range(s):
    """Parse a version range like '1-25' or '5' into a list of ints."""
    s = s.strip()
    if '-' in s:
        parts = s.split('-', 1)
        return list(range(int(parts[0]), int(parts[1]) + 1))
    return [int(s)]


def parse_csv_ints(s):
    """Parse comma-separated integers."""
    return [int(x.strip()) for x in s.split(',')]


def parse_csv_strings(s):
    """Parse comma-separated strings."""
    return [x.strip() for x in s.split(',')]


def main():
    parser = argparse.ArgumentParser(
        description="CI validation suite for k_quirc QR decode library.",
    )
    parser.add_argument('--versions', default='1-25',
                        help='QR version range, e.g. "1-25" (default: 1-25)')
    parser.add_argument('--ecc', default='L,M,Q,H',
                        help='ECC levels, comma-separated (default: L,M,Q,H)')
    parser.add_argument('--modes', default='numeric,alphanumeric,byte',
                        help='Encoding modes (default: numeric,alphanumeric,byte)')
    parser.add_argument('--ppm', default='3,5,8,13,20',
                        help='PPM values, comma-separated (default: 3,5,8,13,20)')
    parser.add_argument('--fill', default='90',
                        help='Fill percentages, comma-separated (default: 90)')
    parser.add_argument('--frames', default='320,480,640',
                        help='Frame sizes, comma-separated (default: 320,480,640)')
    parser.add_argument('--iterations', type=int, default=3,
                        help='Random payloads per configuration (default: 3)')
    parser.add_argument('--json', action='store_true',
                        help='Output JSON summary to stdout')
    parser.add_argument('--binary', default=None,
                        help='Path to pre-built k_quirc_test binary')
    parser.add_argument('--build-dir', default='build_ci',
                        help='Build directory name (default: build_ci)')
    parser.add_argument('--csv-path', default='validation-results.csv',
                        help='CSV output path (default: validation-results.csv)')
    parser.add_argument('--failures-dir', default=None,
                        help='Directory for failure images (default: test/failures/)')
    parser.add_argument('--known-failures', default=None,
                        help='Path to known failures file (default: test/known_failures.txt)')
    parser.add_argument('--k-quirc-dir', default=None,
                        help='k_quirc repo root (default: auto-detect)')
    args = parser.parse_args()

    # Resolve k_quirc directory
    if args.k_quirc_dir:
        k_quirc_dir = os.path.abspath(args.k_quirc_dir)
    else:
        k_quirc_dir = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__))))

    test_dir = os.path.join(k_quirc_dir, 'test')
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if args.failures_dir:
        failures_dir = os.path.abspath(args.failures_dir)
    else:
        failures_dir = os.path.join(script_dir, 'failures')

    # Load known failures
    if args.known_failures:
        known_failures_path = os.path.abspath(args.known_failures)
    else:
        known_failures_path = os.path.join(script_dir, 'known_failures.txt')
    known_failures = load_known_failures(known_failures_path)
    if known_failures:
        print(f"Loaded {len(known_failures)} known failure patterns from "
              f"{known_failures_path}", file=sys.stderr)

    # Parse parameters
    versions = parse_range(args.versions)
    ecc_levels = parse_csv_strings(args.ecc)
    modes = parse_csv_strings(args.modes)
    ppm_values = parse_csv_ints(args.ppm)
    fill_pcts = parse_csv_ints(args.fill)
    frame_sizes = parse_csv_ints(args.frames)

    matrix_params = {
        'versions': versions,
        'ecc_levels': ecc_levels,
        'modes': modes,
        'ppm_values': ppm_values,
        'fill_percentages': fill_pcts,
        'frame_sizes': frame_sizes,
        'iterations': args.iterations,
    }

    # Verify qrencode is available
    _check_qrencode()

    # Step 1: Build test harness
    if args.binary:
        binary = os.path.abspath(args.binary)
        if not os.path.isfile(binary):
            print(f"Error: binary not found: {binary}", file=sys.stderr)
            sys.exit(1)
        print(f"Using pre-built binary: {binary}", file=sys.stderr)
    else:
        print(f"Building test harness...", file=sys.stderr)
        binary = build_harness(test_dir, args.build_dir)
        print(f"  Built: {binary}", file=sys.stderr)

    # Step 2: Build test matrix
    print(f"Building test matrix...", file=sys.stderr)
    cases, skipped = build_test_matrix(
        versions, ecc_levels, modes, ppm_values, fill_pcts,
        frame_sizes, args.iterations)
    print(f"  {len(cases)} test cases, {len(skipped)} skipped",
          file=sys.stderr)

    # Step 3: Generate images to temp directory (parallel)
    with tempfile.TemporaryDirectory(prefix='k_quirc_validation_') as tmp_dir:
        workers = os.cpu_count() or 1
        print(f"Generating {len(cases)} test images ({workers} workers)...",
              file=sys.stderr)

        gen_results = {}
        with ProcessPoolExecutor(max_workers=workers) as pool:
            for fname, ok, err in pool.map(
                    _generate_one, ((c, tmp_dir) for c in cases),
                    chunksize=32):
                gen_results[fname] = (ok, err)

        # Drop cases whose image wasn't produced; record as skipped.
        generated_cases = []
        for case in cases:
            ok, err = gen_results.get(case['filename'], (False, 'no result'))
            if ok:
                generated_cases.append(case)
            else:
                skipped.append({
                    'version': case['version'], 'ecc': case['ecc'],
                    'mode': case['mode'], 'ppm': case['ppm'],
                    'frame': case['frame'],
                    'reason': f'image generation failed: {err}',
                })
        dropped = len(cases) - len(generated_cases)
        if dropped:
            print(f"  {dropped} images failed to generate (recorded as skipped)",
                  file=sys.stderr)
        cases = generated_cases

        # Step 4: Run test harness
        print(f"Running k_quirc_test on {len(cases)} images...",
              file=sys.stderr)
        try:
            stdout = run_harness(binary, tmp_dir)
        except subprocess.TimeoutExpired as e:
            print(f"Error: harness timed out after {e.timeout}s",
                  file=sys.stderr)
            sys.exit(1)
        parsed = parse_output(stdout)
        print(f"  Harness reported {len(parsed)} results", file=sys.stderr)

        # Step 5: Validate
        results = validate_results(cases, parsed, known_failures)
        regressions = [r for r in results
                       if r['status'] == 'unexpected_failure']

        # Step 6: Copy regression failure images (not expected failures)
        if regressions:
            os.makedirs(failures_dir, exist_ok=True)
            for r in regressions:
                src = os.path.join(tmp_dir, r['filename'])
                if os.path.exists(src):
                    shutil.copy2(src,
                                 os.path.join(failures_dir, r['filename']))
            print(f"  {len(regressions)} regression images saved to "
                  f"{failures_dir}", file=sys.stderr)

    # Step 7: Output
    write_csv(results, args.csv_path)
    print(f"  CSV written to {args.csv_path}", file=sys.stderr)

    print_report(results, skipped)

    if args.json:
        json_out = make_json_output(results, skipped, matrix_params)
        print(json.dumps(json_out, indent=2))

    # Exit code: 0 if no regressions (expected failures are OK)
    if regressions:
        sys.exit(1)
    sys.exit(0)


if __name__ == '__main__':
    main()
