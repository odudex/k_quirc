#!/usr/bin/env python3
"""Physically-ordered camera degradation for synthetic QR test vectors.

Pipeline order mirrors a real capture:
    scene -> optics (defocus / motion blur) -> illumination -> sensor
             (exposure gain, clipping) -> noise -> quantisation

Blur is specified in PIXELS, not modules: real defocus is fixed by the optics,
so its effect on a QR grows as pixels-per-module shrinks.  That coupling is
what the current all-sharp corpus completely lacks.
"""
import numpy as np
from PIL import Image


def _gauss_kernel(sigma, radius=None):
    if radius is None:
        radius = max(1, int(np.ceil(3 * sigma)))
    x = np.arange(-radius, radius + 1, dtype=np.float64)
    k = np.exp(-(x ** 2) / (2.0 * sigma ** 2))
    return k / k.sum()


def _sep_convolve(img, k):
    pad = len(k) // 2
    a = np.pad(img, pad, mode='edge')
    out = np.empty_like(img, dtype=np.float64)
    tmp = np.zeros((img.shape[0], img.shape[1] + 2 * pad), dtype=np.float64)
    for i, w in enumerate(k):
        tmp += w * a[i:i + img.shape[0], :]
    out[:] = 0.0
    for i, w in enumerate(k):
        out += w * tmp[:, i:i + img.shape[1]]
    return out


def defocus(img, sigma_px):
    """Gaussian defocus, sigma in pixels."""
    if sigma_px <= 0:
        return img
    return _sep_convolve(img, _gauss_kernel(sigma_px))


def _minmax_filter(img, radius, take_min):
    """Square-structuring-element morphology, radius in whole pixels."""
    out = img
    for _ in range(radius):
        shifted = [out,
                   np.pad(out, ((0, 0), (1, 0)), mode='edge')[:, :-1],
                   np.pad(out, ((0, 0), (0, 1)), mode='edge')[:, 1:],
                   np.pad(out, ((1, 0), (0, 0)), mode='edge')[:-1, :],
                   np.pad(out, ((0, 1), (0, 0)), mode='edge')[1:, :]]
        out = np.minimum.reduce(shifted) if take_min else \
            np.maximum.reduce(shifted)
    return out


def ink_spread(img, px):
    """Grow (px > 0) or shrink (px < 0) dark features, in pixels.

    Toner and ink bleed outward on paper, so a printed QR carries slightly
    fat black modules before the camera ever sees it.  This is the one common
    distortion that pushes the binarization the opposite way from defocus and
    overexposure, so a threshold controller that can only ever raise its
    offset will fail on it.
    """
    if px == 0.0:
        return img
    whole = int(np.floor(abs(px)))
    frac = abs(px) - whole
    take_min = px > 0
    out = _minmax_filter(img, whole, take_min) if whole else img
    if frac > 0:
        out = out * (1.0 - frac) + _minmax_filter(out, 1, take_min) * frac
    return out


def _homography(src, dst):
    """3x3 H mapping src -> dst, from four point correspondences."""
    rows = []
    for (sx, sy), (dx, dy) in zip(src, dst):
        rows.append([sx, sy, 1, 0, 0, 0, -dx * sx, -dx * sy])
        rows.append([0, 0, 0, sx, sy, 1, -dy * sx, -dy * sy])
    a = np.array(rows, dtype=np.float64)
    b = np.array([c for p in dst for c in p], dtype=np.float64)
    h = np.linalg.solve(a, b)
    return np.append(h, 1.0).reshape(3, 3)


def _rotation(rx, ry, rz):
    ax, ay, az = np.deg2rad([rx, ry, rz])
    cx, sx = np.cos(ax), np.sin(ax)
    cy, sy = np.cos(ay), np.sin(ay)
    cz, sz = np.cos(az), np.sin(az)
    mx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    my = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    mz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return mx @ my @ mz


def perspective(img, rx=0.0, ry=0.0, rz=0.0, focal=3.0, fill=None):
    """Pinhole projection of the image plane tilted by (rx, ry, rz) degrees.

    Handheld capture is never fronto-parallel, and a tilted QR is what forces
    the decoder's perspective fit to work.  The warp is renormalised so the
    subject keeps its extent, leaving pixels-per-module roughly unchanged.
    """
    if rx == 0.0 and ry == 0.0 and rz == 0.0:
        return img

    h, w = img.shape
    if fill is None:
        fill = float(img[0, 0])

    corners = np.array([[-.5, -.5], [.5, -.5], [.5, .5], [-.5, .5]])
    pts = np.c_[corners, np.zeros(4)] @ _rotation(rx, ry, rz).T
    pts[:, 2] += focal
    proj = pts[:, :2] * focal / pts[:, 2:3]
    extent = max(np.ptp(proj[:, 0]), np.ptp(proj[:, 1]))
    proj = proj / extent
    proj = proj - proj.mean(axis=0)

    dst = (proj + 0.5) * np.array([w, h])
    src = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float64)
    inv = _homography(dst, src)

    yy, xx = np.mgrid[0:h, 0:w]
    den = inv[2, 0] * xx + inv[2, 1] * yy + inv[2, 2]
    den = np.where(np.abs(den) < 1e-12, 1e-12, den)
    sx = (inv[0, 0] * xx + inv[0, 1] * yy + inv[0, 2]) / den
    sy = (inv[1, 0] * xx + inv[1, 1] * yy + inv[1, 2]) / den

    x0 = np.floor(sx).astype(np.int64)
    y0 = np.floor(sy).astype(np.int64)
    fx = sx - x0
    fy = sy - y0
    valid = (x0 >= 0) & (x0 < w - 1) & (y0 >= 0) & (y0 < h - 1)
    xc = np.clip(x0, 0, w - 2)
    yc = np.clip(y0, 0, h - 2)
    out = (img[yc, xc] * (1 - fx) * (1 - fy) +
           img[yc, xc + 1] * fx * (1 - fy) +
           img[yc + 1, xc] * (1 - fx) * fy +
           img[yc + 1, xc + 1] * fx * fy)
    return np.where(valid, out, fill)


def motion_blur(img, length_px, angle_deg=0.0):
    """Linear motion smear of the given length (pixels) and direction."""
    if length_px <= 1:
        return img
    n = int(round(length_px))
    th = np.deg2rad(angle_deg)
    dx, dy = np.cos(th), np.sin(th)
    acc = np.zeros_like(img, dtype=np.float64)
    h, w = img.shape
    yy, xx = np.mgrid[0:h, 0:w]
    for i in range(n):
        t = i - (n - 1) / 2.0
        sx = np.clip(np.round(xx + t * dx).astype(int), 0, w - 1)
        sy = np.clip(np.round(yy + t * dy).astype(int), 0, h - 1)
        acc += img[sy, sx]
    return acc / n


def illumination(img, gradient=0.0, angle_deg=0.0, vignette=0.0):
    """Multiplicative lighting: linear gradient (+/- fraction) and vignetting."""
    h, w = img.shape
    yy, xx = np.mgrid[0:h, 0:w]
    g = np.ones((h, w), dtype=np.float64)
    if gradient:
        th = np.deg2rad(angle_deg)
        u = (xx / max(w - 1, 1) - 0.5) * np.cos(th) + \
            (yy / max(h - 1, 1) - 0.5) * np.sin(th)
        g *= 1.0 + gradient * 2.0 * u
    if vignette:
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        r2 = ((xx - cx) ** 2 + (yy - cy) ** 2) / (cx ** 2 + cy ** 2)
        g *= 1.0 - vignette * r2
    return img * g


def expose(img, ev=0.0, black_level=0.0, white_ceiling=255.0):
    """Sensor response: EV gain about the scene, then range compression.

    black_level lifts the darkest achievable value (veiling glare / flare);
    white_ceiling caps the brightest (underexposure).  Both compress contrast
    the way real captures do.
    """
    y = img * (2.0 ** ev)
    y = black_level + y * (white_ceiling - black_level) / 255.0
    return y


def add_noise(img, sigma, rng):
    if sigma <= 0:
        return img
    return img + rng.normal(0.0, sigma, img.shape)


def degrade(gray_u8, *, ink=0.0, rx=0.0, ry=0.0, rz=0.0, focal=3.0,
            sigma_px=0.0, motion_px=0.0, motion_angle=0.0,
            gradient=0.0, gradient_angle=0.0, vignette=0.0,
            ev=0.0, black_level=0.0, white_ceiling=255.0,
            noise=0.0, gamma=2.2, seed=0):
    """Apply the full pipeline to a uint8 grayscale array.

    Geometry, optics and illumination all act on LINEAR light; the gamma
    encode happens in the sensor, after exposure and clipping.  Blurring or
    resampling gamma-encoded pixels instead (the naive approach) biases edges
    the wrong way and produces black dilation where a real camera produces
    black erosion.

    Ink spread belongs to the printed subject, so it is applied before the
    viewing geometry.  The geometry comes next: the scene is projected by the
    lens, and only then blurred by its point spread function.
    """
    rng = np.random.default_rng(seed)
    x = (gray_u8.astype(np.float64) / 255.0) ** gamma      # -> linear light
    x = ink_spread(x, ink)
    x = perspective(x, rx, ry, rz, focal)
    x = defocus(x, sigma_px)
    if motion_px > 1:
        x = motion_blur(x, motion_px, motion_angle)
    if gradient or vignette:
        x = illumination(x, gradient, gradient_angle, vignette)
    x = x * (2.0 ** ev)                                    # exposure, linear
    x = np.clip(x, 0.0, 1.0)                               # highlight clipping
    x = x ** (1.0 / gamma)                                 # sensor encode
    x = black_level + x * (white_ceiling - black_level)    # flare / range
    x = add_noise(x, noise, rng)
    return np.clip(x, 0, 255).astype(np.uint8)


def degrade_file(src, dst, **kw):
    a = np.asarray(Image.open(src).convert('L'))
    Image.fromarray(degrade(a, **kw)).save(dst)
