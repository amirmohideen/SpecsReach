"""Orientation math for the bridge: RPY <-> rotation matrix and geodesic steps.

Poses on the wire and in the drivers carry orientation as roll/pitch/yaw
degrees with R = Rz(yaw) * Ry(pitch) * Rx(roll) in the arm base frame (the
xArm and AUBO convention, live-verified). RPY is only an ENCODING: near
pitch +/-90 deg the roll/yaw split is singular, so anything that ramps,
clamps or compares orientation must work on the rotation itself. This module
does that with rotation vectors (axis * angle, degrees, base frame):

- `rotvec_between(a, b)`  the base-frame rotation vector taking a to b
                          (log(R_b * R_a^T)), shortest way round
- `apply_rotvec(rpy, w)`  rotate a pose by w in the base frame (exp(w) * R)
- `angle_between(a, b)`   geodesic angle, degrees

`matrix_to_rpy(R, near=...)` picks, of the two RPY triples that encode R,
the one closest to `near` and unwraps it toward `near`, so a stream of
commanded poses stays numerically continuous (no 180/-180 or branch flips
reaching IK seeds or logs). Pure Python: called a few times per servo tick.
"""

from __future__ import annotations

import math

Matrix = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
Vec3 = tuple[float, float, float]

_GIMBAL_EPS = 1e-9


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> Matrix:
    """R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in degrees."""
    r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _wrap(deg: float) -> float:
    """Into (-180, 180]."""
    w = math.fmod(deg + 180.0, 360.0)
    if w <= 0.0:
        w += 360.0
    return w - 180.0


def _unwrap_toward(value: float, ref: float) -> float:
    """value + k*360 closest to ref."""
    return ref + _wrap(value - ref)


def matrix_to_rpy(m: Matrix, near: Vec3 | None = None) -> Vec3:
    """RPY degrees encoding `m`. Without `near`: the principal triple, each in
    (-180, 180], pitch in [-90, 90]. With `near`: of the two triples encoding
    `m` (and, at gimbal lock, the whole family), the one closest to `near`,
    unwrapped toward it."""
    r20 = max(-1.0, min(1.0, m[2][0]))
    if 1.0 - abs(r20) < _GIMBAL_EPS:
        # Pitch = -/+90: only roll -/+ yaw is determined. Keep the reference
        # roll (or 0) and give yaw the rest.
        pitch = -90.0 if r20 > 0.0 else 90.0
        roll = near[0] if near is not None else 0.0
        if pitch > 0.0:
            # R = Rz(y) Ry(90) Rx(r): m01 = sin(r - y), m11 = cos(r - y)
            diff = math.degrees(math.atan2(m[0][1], m[1][1]))   # r - y
            yaw = roll - diff
        else:
            # R = Rz(y) Ry(-90) Rx(r): m01 = -sin(r + y), m11 = cos(r + y)
            total = math.degrees(math.atan2(-m[0][1], m[1][1]))  # r + y
            yaw = total - roll
        cand = [(roll, pitch, yaw)]
    else:
        pitch = math.degrees(math.asin(-r20))
        roll = math.degrees(math.atan2(m[2][1], m[2][2]))
        yaw = math.degrees(math.atan2(m[1][0], m[0][0]))
        cand = [(roll, pitch, yaw),
                (roll + 180.0, 180.0 - pitch, yaw + 180.0)]
    if near is None:
        r, p, y = cand[0]
        return (_wrap(r), p, _wrap(y))

    def unwrapped(t):
        return tuple(_unwrap_toward(v, n) for v, n in zip(t, near))

    best = min((unwrapped(c) for c in cand),
               key=lambda t: sum((v - n) ** 2 for v, n in zip(t, near)))
    return best


def _mul(a: Matrix, b: Matrix) -> Matrix:
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))  # type: ignore[return-value]


def _transpose(a: Matrix) -> Matrix:
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def matrix_log(m: Matrix) -> Vec3:
    """Rotation vector (axis * angle, degrees) of rotation matrix m, angle in
    [0, 180]."""
    trace = m[0][0] + m[1][1] + m[2][2]
    c = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    angle = math.acos(c)
    if angle < 1e-9:
        # Small angle: log ~ vee(m - m^T) / 2.
        return (math.degrees((m[2][1] - m[1][2]) / 2.0),
                math.degrees((m[0][2] - m[2][0]) / 2.0),
                math.degrees((m[1][0] - m[0][1]) / 2.0))
    if math.pi - angle < 1e-6:
        # Near 180 deg: axis from the symmetric part (sin ~ 0 is unusable).
        xx = max(0.0, (m[0][0] + 1.0) / 2.0)
        yy = max(0.0, (m[1][1] + 1.0) / 2.0)
        zz = max(0.0, (m[2][2] + 1.0) / 2.0)
        x, y, z = math.sqrt(xx), math.sqrt(yy), math.sqrt(zz)
        # Fix relative signs from the off-diagonal terms (largest component +).
        if x >= y and x >= z:
            y = math.copysign(y, m[0][1] + m[1][0])
            z = math.copysign(z, m[0][2] + m[2][0])
        elif y >= z:
            x = math.copysign(x, m[0][1] + m[1][0])
            z = math.copysign(z, m[1][2] + m[2][1])
        else:
            x = math.copysign(x, m[0][2] + m[2][0])
            y = math.copysign(y, m[1][2] + m[2][1])
        # Overall sign: the antisymmetric part is 2 sin(angle) * axis -- tiny
        # here but still signed, and it separates angle from 360 - angle.
        vee = (m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1])
        if x * vee[0] + y * vee[1] + z * vee[2] < 0.0:
            x, y, z = -x, -y, -z
        n = math.sqrt(x * x + y * y + z * z) or 1.0
        deg = math.degrees(angle)
        return (x / n * deg, y / n * deg, z / n * deg)
    s = 2.0 * math.sin(angle)
    axis = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    deg = math.degrees(angle)
    return (axis[0] * deg, axis[1] * deg, axis[2] * deg)


def matrix_exp(w: Vec3) -> Matrix:
    """Rotation matrix of rotation vector w (degrees), Rodrigues."""
    angle = math.radians(math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]))
    if angle < 1e-12:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    x, y, z = (math.radians(c) / angle for c in w)
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return ((c + x * x * t, x * y * t - z * s, x * z * t + y * s),
            (y * x * t + z * s, c + y * y * t, y * z * t - x * s),
            (z * x * t - y * s, z * y * t + x * s, c + z * z * t))


def rotvec_between(a: Vec3, b: Vec3) -> Vec3:
    """Base-frame rotation vector (degrees) taking orientation a to b (RPY
    triples): log(R_b * R_a^T). Its norm is the geodesic angle, <= 180."""
    ra, rb = rpy_to_matrix(*a), rpy_to_matrix(*b)
    return matrix_log(_mul(rb, _transpose(ra)))


def angle_between(a: Vec3, b: Vec3) -> float:
    w = rotvec_between(a, b)
    return math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2])


def apply_rotvec(rpy: Vec3, w: Vec3) -> Vec3:
    """Orientation rpy rotated by base-frame rotation vector w (degrees):
    exp(w) * R(rpy), encoded continuously near `rpy`."""
    return matrix_to_rpy(_mul(matrix_exp(w), rpy_to_matrix(*rpy)), near=rpy)
