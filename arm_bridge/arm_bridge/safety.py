"""Server-owned safety backstop (rev 3 sections 1 and 4).

Two responsibilities, kept separate:

  * `validate_target` — REJECT a commanded target that is non-finite or outside
    the hard workspace/orientation box. Rejected targets never move the arm; the
    packet is dropped (rev 3 section 4: "reject ... non-finite floats, and
    targets outside server-owned hard limits").

  * `clamp_step` / `clamp_gripper` — for an *accepted* target, limit how far the
    arm may move in one servo tick so combined multi-axis motion stays at or
    below `safety_factor` of the configured velocity limits.

  * `posture_violation` — why the arm's ACTUAL posture (joints + TCP) is
    outside the envelope (bridge joint limits, box, reach, keep-outs). Used
    to refuse SET HOME and to block teleop engage after free-drive.

  * `retreat_joints` / `retreat_pose` — the nearest in-envelope joint vector /
    TCP point for the operator-triggered RETREAT move (never automatic).

All values are xArm base frame, mm and degrees.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from . import rotation
from .config import ArmBridgeConfig, Keepout, WorkspaceLimits

Pose = tuple[float, float, float, float, float, float]  # x,y,z,roll,pitch,yaw


@dataclass(frozen=True)
class Rejection:
    reason: str


def _all_finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


def validate_target(pose: Pose, gripper: float, limits: WorkspaceLimits) -> Rejection | None:
    """Return a Rejection if the target must be refused, else None."""
    x, y, z, roll, pitch, yaw = pose
    if not _all_finite(x, y, z, roll, pitch, yaw, gripper):
        return Rejection("non-finite target value")
    if not (limits.x_min <= x <= limits.x_max):
        return Rejection(f"x {x:.1f} outside [{limits.x_min}, {limits.x_max}]")
    if not (limits.y_min <= y <= limits.y_max):
        return Rejection(f"y {y:.1f} outside [{limits.y_min}, {limits.y_max}]")
    if not (limits.z_min <= z <= limits.z_max):
        return Rejection(f"z {z:.1f} outside [{limits.z_min}, {limits.z_max}]")
    if not (limits.roll_min <= roll <= limits.roll_max):
        return Rejection(f"roll {roll:.1f} outside [{limits.roll_min}, {limits.roll_max}]")
    if not (limits.pitch_min <= pitch <= limits.pitch_max):
        return Rejection(f"pitch {pitch:.1f} outside [{limits.pitch_min}, {limits.pitch_max}]")
    if not (limits.yaw_min <= yaw <= limits.yaw_max):
        return Rejection(f"yaw {yaw:.1f} outside [{limits.yaw_min}, {limits.yaw_max}]")
    # Reach sphere: the box is a cuboid but the arm's reach is a sphere, so a box
    # corner can be unreachable -- reject a TCP beyond the shoulder-centred reach
    # radius before the controller grinds the elbow into its limit (error 23).
    if limits.reach_max_mm > 0.0:
        dz = z - limits.reach_center_z
        reach = math.sqrt(x * x + y * y + dz * dz)
        if reach > limits.reach_max_mm:
            return Rejection(
                f"reach {reach:.0f} exceeds {limits.reach_max_mm:.0f} mm "
                f"(target outside the arm's reachable sphere)")
    # Keep-out zones: a soft wall like the box edge -- the target is dropped and
    # the arm holds at its last good one until the hand comes back out.
    zone = keepout_hit(x, y, z, limits)
    if zone is not None:
        return Rejection(f"target inside keep-out '{zone.label}'")
    # Gripper is clamped, not rejected, so only its finiteness matters here.
    return None


def keepout_hit(x: float, y: float, z: float, limits: WorkspaceLimits) -> Keepout | None:
    """The first keep-out zone containing (x, y, z), else None."""
    for zone in limits.keepouts:
        if zone.contains(x, y, z):
            return zone
    return None


# ---- actual-posture envelope (SET HOME / teleop engage / RETREAT) -----------

def position_violation(x: float, y: float, z: float, limits: WorkspaceLimits) -> str | None:
    if not _all_finite(x, y, z):
        return "TCP position is not finite"
    if not (limits.x_min <= x <= limits.x_max):
        return f"TCP x {x:.0f} mm is outside [{limits.x_min:.0f}, {limits.x_max:.0f}]"
    if not (limits.y_min <= y <= limits.y_max):
        return f"TCP y {y:.0f} mm is outside [{limits.y_min:.0f}, {limits.y_max:.0f}]"
    if not (limits.z_min <= z <= limits.z_max):
        return f"TCP z {z:.0f} mm is outside [{limits.z_min:.0f}, {limits.z_max:.0f}]"
    if limits.reach_max_mm > 0.0:
        dz = z - limits.reach_center_z
        reach = math.sqrt(x * x + y * y + dz * dz)
        if reach > limits.reach_max_mm:
            return f"TCP reach {reach:.0f} mm exceeds {limits.reach_max_mm:.0f}"
    zone = keepout_hit(x, y, z, limits)
    if zone is not None:
        return f"TCP is inside keep-out '{zone.label}'"
    return None


def joint_violation(joints_deg, joint_limits_deg) -> str | None:
    """First joint outside the bridge-enforced limits, else None. Either side
    None/empty = no joint check (the backend has no joint stream or limits)."""
    if not joints_deg or not joint_limits_deg:
        return None
    for i, (q, (lo, hi)) in enumerate(zip(joints_deg, joint_limits_deg)):
        if not math.isfinite(q):
            return f"J{i + 1} angle is not finite"
        if not lo <= q <= hi:
            return f"J{i + 1} at {q:.1f} deg is outside [{lo:.0f}, {hi:.0f}]"
    return None


def posture_violation(pose: Pose, joints_deg, joint_limits_deg,
                      limits: WorkspaceLimits) -> str | None:
    """Why the arm's ACTUAL posture is outside the envelope, or None when it
    is inside. Joints first (a folded elbow is the usual cause), then the TCP
    against the box, the reach sphere and the keep-outs. Orientation limits
    are not applied to the actual pose (an RPY readback can sit a hair past
    +/-180 and the Quest never anchors on orientation limits)."""
    reason = joint_violation(joints_deg, joint_limits_deg)
    if reason is not None:
        return reason
    return position_violation(pose[0], pose[1], pose[2], limits)


def retreat_joints(joints_deg, joint_limits_deg, margin_deg: float) -> list[float] | None:
    """Nearest joint vector inside the bridge limits (each offending joint
    clamped to its limit plus `margin_deg` inward); None when every joint is
    already inside."""
    if not joints_deg or not joint_limits_deg:
        return None
    out, changed = [], False
    for q, (lo, hi) in zip(joints_deg, joint_limits_deg):
        inner_lo, inner_hi = min(lo + margin_deg, hi), max(hi - margin_deg, lo)
        if q < lo:
            out.append(inner_lo)
            changed = True
        elif q > hi:
            out.append(inner_hi)
            changed = True
        else:
            out.append(q)
    return out if changed else None


def _push_out_of_zone(x: float, y: float, z: float, zone: Keepout, margin: float,
                      limits: WorkspaceLimits) -> tuple[float, float, float]:
    """Move (x, y, z) out of `zone` by the shortest displacement that stays
    inside the workspace box, plus `margin`."""
    def inside_box(px, py, pz) -> bool:
        return (limits.x_min <= px <= limits.x_max and limits.y_min <= py <= limits.y_max
                and limits.z_min <= pz <= limits.z_max)

    candidates: list[tuple[float, tuple[float, float, float]]] = []
    if zone.shape == "sphere":
        dx, dy, dz = x - zone.cx, y - zone.cy, z - zone.cz
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm < 1e-6:
            dx, dy, dz, norm = 0.0, 0.0, 1.0, 1.0       # at the centre: go up
        scale = (zone.radius + margin) / norm
        candidates.append((zone.radius + margin - norm,
                           (zone.cx + dx * scale, zone.cy + dy * scale, zone.cz + dz * scale)))
    elif zone.shape == "cylinder":
        dx, dy = x - zone.cx, y - zone.cy
        rho = math.sqrt(dx * dx + dy * dy)
        if rho < 1e-6:
            dx, dy, rho = 1.0, 0.0, 1.0
        scale = (zone.radius + margin) / rho
        candidates.append((zone.radius + margin - rho,
                           (zone.cx + dx * scale, zone.cy + dy * scale, z)))
        candidates.append((zone.z_max + margin - z, (x, y, zone.z_max + margin)))
        candidates.append((z - (zone.z_min - margin), (x, y, zone.z_min - margin)))
    else:                                               # box: out through a face
        candidates.extend([
            (zone.x_max + margin - x, (zone.x_max + margin, y, z)),
            (x - (zone.x_min - margin), (zone.x_min - margin, y, z)),
            (zone.y_max + margin - y, (x, zone.y_max + margin, z)),
            (y - (zone.y_min - margin), (x, zone.y_min - margin, z)),
            (zone.z_max + margin - z, (x, y, zone.z_max + margin)),
            (z - (zone.z_min - margin), (x, y, zone.z_min - margin)),
        ])
    # Prefer an exit that lands inside the box AND clear of every other zone
    # (fence + rack overlap is the normal case), then one inside the box.
    clear = [c for c in candidates
             if inside_box(*c[1]) and keepout_hit(*c[1], limits) is None]
    feasible = clear or [c for c in candidates if inside_box(*c[1])] or candidates
    feasible.sort(key=lambda c: c[0])
    return feasible[0][1]


def retreat_pose(pose: Pose, limits: WorkspaceLimits, margin_mm: float) -> Pose | None:
    """Nearest TCP point inside the box / reach sphere / outside every keep-out,
    orientation unchanged; None when the TCP is already inside. Pushes out of
    zones one at a time (a few passes), then re-applies the box and reach."""
    x, y, z = pose[0], pose[1], pose[2]
    if position_violation(x, y, z, limits) is None:
        return None
    for _ in range(4):
        x = min(max(x, limits.x_min + margin_mm), limits.x_max - margin_mm)
        y = min(max(y, limits.y_min + margin_mm), limits.y_max - margin_mm)
        z = min(max(z, limits.z_min + margin_mm), limits.z_max - margin_mm)
        if limits.reach_max_mm > 0.0:
            dz = z - limits.reach_center_z
            reach = math.sqrt(x * x + y * y + dz * dz)
            if reach > limits.reach_max_mm - margin_mm and reach > 1e-6:
                scale = (limits.reach_max_mm - margin_mm) / reach
                x, y, z = x * scale, y * scale, limits.reach_center_z + dz * scale
        zone = keepout_hit(x, y, z, limits)
        if zone is None:
            break
        x, y, z = _push_out_of_zone(x, y, z, zone, margin_mm, limits)
    return (x, y, z, pose[3], pose[4], pose[5])


def _wrap_deg(delta: float) -> float:
    """Shortest signed angular delta in (-180, 180], so 179 -> -179 rotates 2
    degrees, not 358 -- prevents a wrap from commanding a full spin."""
    return (delta + 180.0) % 360.0 - 180.0


def _norm(components: tuple[float, ...]) -> float:
    return math.sqrt(sum(c * c for c in components))


def clamp_step(current: Pose, target: Pose, dt: float, config: ArmBridgeConfig) -> Pose:
    """Return the pose to command this tick: `current` moved toward `target`, with
    linear and angular motion each capped by norm at safety_factor * limit * dt.

    Linear (x,y,z) and angular motion are clamped independently so a fast
    rotation cannot borrow the translation budget. Rotation is geodesic: the
    step turns about the single base-frame axis that takes the current
    orientation to the target, the short way round -- RPY numbers are only an
    encoding and are singular near pitch +/-90 (rotation.py)."""
    factor = config.safety_factor
    max_lin_step = factor * config.velocity.max_linear_mm_s * dt
    max_ang_step = factor * config.velocity.max_angular_deg_s * dt

    lin_delta = (target[0] - current[0], target[1] - current[1], target[2] - current[2])
    lin_delta = _scale_to(lin_delta, max_lin_step)
    rot = _scale_to(rotation.rotvec_between(current[3:6], target[3:6]), max_ang_step)
    rpy = rotation.apply_rotvec(current[3:6], rot)
    return (current[0] + lin_delta[0], current[1] + lin_delta[1], current[2] + lin_delta[2],
            rpy[0], rpy[1], rpy[2])


def pose_distance(a: Pose, b: Pose) -> tuple[float, float]:
    """(translation mm, geodesic rotation deg) between two poses."""
    lin = math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))
    return lin, rotation.angle_between(a[3:6], b[3:6])


def _scale_to(delta: tuple[float, ...], max_norm: float) -> tuple[float, ...]:
    norm = _norm(delta)
    if norm <= max_norm or norm == 0.0:
        return delta
    scale = max_norm / norm
    return tuple(c * scale for c in delta)


# (linear mm/s, angular velocity deg/s) -- both base-frame vectors; the
# angular part is a rotation-vector rate (axis * deg/s), NOT RPY rates.
Velocity = tuple[tuple[float, float, float], tuple[float, float, float]]
ZERO_VELOCITY: Velocity = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def brake_speed(dist: float, a_max: float, dt: float) -> float:
    """The fastest speed from which a discrete decel of a_max*dt per tick
    stops exactly after `dist` (see `_ramp`): n = the largest integer with
    n(n+1)/2 <= D (D in units of a*dt^2), plus the remainder's share."""
    if dist <= 0.0:
        return 0.0
    units = dist / (a_max * dt * dt)
    n = int((math.sqrt(1.0 + 8.0 * units) - 1.0) / 2.0)
    while (n + 1) * (n + 2) / 2.0 <= units:      # float guard
        n += 1
    while n > 0 and n * (n + 1) / 2.0 > units:
        n -= 1
    return a_max * dt * (n + (units - n * (n + 1) / 2.0) / (n + 1))


def joint_path_profile(points, v_max: float, a_max: float, dt: float) -> list[list[float]]:
    """Time-parameterize a joint-space polyline (deg) for streaming at `dt`:
    one point per tick (points[0], the start, excluded), starting from rest,
    path speed <= v_max and changing by <= a_max*dt per tick, ending at rest
    exactly ON the last point (the discrete braking curve). Path length is the
    max-norm, so v_max and a_max bound every joint. A turn sharp enough that
    passing it at full speed would change some joint's velocity by more than
    a_max*dt -- a recorded hand wiggle reverses -- is a stop: the path splits
    there into runs, each from rest to rest, with one still tick between.
    BACKOUT's joint replay."""
    pts = [[float(v) for v in p] for p in points]
    pts = [p for k, p in enumerate(pts) if k == 0 or p != pts[k - 1]]
    if len(pts) < 2 or v_max <= 0.0 or a_max <= 0.0:
        return []
    seg = [max(abs(b - a) for a, b in zip(p, q)) for p, q in zip(pts, pts[1:])]
    unit = [[(b - a) / ln for a, b in zip(p, q)] for p, q, ln in zip(pts, pts[1:], seg)]
    runs = [[pts[0]]]
    for k in range(1, len(pts) - 1):
        runs[-1].append(pts[k])
        turn = max(abs(b - a) for a, b in zip(unit[k - 1], unit[k]))
        if turn * v_max > a_max * dt:
            runs.append([pts[k]])
    runs[-1].append(pts[-1])
    out: list[list[float]] = []
    for run in runs:
        if out:
            out.append(list(out[-1]))               # one still tick at the turn
        out += _profile_run(run, v_max, a_max, dt)
    return out


def _profile_run(pts: list[list[float]], v_max: float, a_max: float,
                 dt: float) -> list[list[float]]:
    """One rest-to-rest run of `joint_path_profile` (bends gentle enough to
    pass at full speed)."""
    seg = [max(abs(b - a) for a, b in zip(p, q)) for p, q in zip(pts, pts[1:])]
    total = sum(seg)
    out: list[list[float]] = []
    s = v = base = 0.0
    i = 0
    while total - s > 1e-9 and len(out) < 1_000_000:
        v = min(v_max, v + a_max * dt, brake_speed(total - s, a_max, dt), (total - s) / dt)
        s = min(total, s + v * dt)
        while i < len(seg) - 1 and s > base + seg[i]:
            base += seg[i]
            i += 1
        f = min(max((s - base) / seg[i], 0.0), 1.0)
        out.append([a + (b - a) * f for a, b in zip(pts[i], pts[i + 1])])
    if out:
        out[-1] = list(pts[-1])                     # float guard: exactly on the end
    return out


def _ramp(delta: tuple[float, ...], v_prev: tuple[float, ...], v_max: float,
          a_max: float, dt: float) -> tuple[float, ...]:
    """Velocity for this tick toward `delta` away: at most v_max, slow enough
    to stop exactly on the target with a_max, never past it in one tick, and
    changed from v_prev by at most a_max*dt (vector norm).

    Discrete braking curve, in units of a*dt (speed) and a*dt^2 (distance
    D): the largest speed s = n + f whose decel sequence s, s-1, ..., f sums
    to exactly D -- n = the largest integer with n(n+1)/2 <= D, f = the
    remainder / (n+1). Following it needs exactly a*dt of decel per tick and
    lands on the target (the continuous sqrt(2ad) curve overshoots by a
    fraction of a step)."""
    dist = _norm(delta)
    if dist > 0.0:
        speed = min(v_max, brake_speed(dist, a_max, dt), dist / dt)
        v_des = tuple(c / dist * speed for c in delta)
    else:
        v_des = (0.0,) * len(delta)
    dv = _scale_to(tuple(d - p for d, p in zip(v_des, v_prev)), a_max * dt)
    return tuple(p + d for p, d in zip(v_prev, dv))


def accel_step(current: Pose, velocity: Velocity, target: Pose, dt: float,
               config: ArmBridgeConfig) -> tuple[Pose, Velocity]:
    """Like `clamp_step`, plus an acceleration limit: returns the pose to
    command this tick and the commanded velocity (linear mm/s, angular deg/s
    as a base-frame rotation-vector rate -- feed it back next tick). Speed <=
    safety_factor x the velocity limits, every velocity change <=
    safety_factor x the acceleration limits, and a stationary target is
    reached without overshoot. Rotation ramps along the geodesic (the error
    is the rotation vector from current to target), so the path and rate are
    right at any orientation, including pitch +/-90 where RPY is singular. A
    target that REVERSES mid-move can be overshot by up to v^2/2a along the
    old direction, so the caller checks the result against the position
    envelope (`position_violation`)."""
    factor = config.safety_factor
    vel = config.velocity
    lin_delta = (target[0] - current[0], target[1] - current[1], target[2] - current[2])
    ang_delta = rotation.rotvec_between(current[3:6], target[3:6])
    lin_v = _ramp(lin_delta, velocity[0], factor * vel.max_linear_mm_s,
                  factor * vel.max_linear_acc_mm_s2, dt)
    ang_v = _ramp(ang_delta, velocity[1], factor * vel.max_angular_deg_s,
                  factor * vel.max_angular_acc_deg_s2, dt)
    rpy = rotation.apply_rotvec(current[3:6], tuple(w * dt for w in ang_v))
    pose = (current[0] + lin_v[0] * dt, current[1] + lin_v[1] * dt, current[2] + lin_v[2] * dt,
            rpy[0], rpy[1], rpy[2])
    return pose, (lin_v, ang_v)


def _stop_path_clear(p: tuple[float, float, float], v: tuple[float, float, float],
                     a_max: float, limits: WorkspaceLimits, samples: int = 4) -> bool:
    """True when p, and the straight braking path from p at velocity v under
    deceleration a_max, stay inside the position envelope (box, reach
    sphere, keep-outs). Sampled: the envelope is not convex (keep-outs)."""
    if position_violation(p[0], p[1], p[2], limits) is not None:
        return False
    sp = _norm(v)
    if sp <= 0.0:
        return True
    dist = sp * sp / (2.0 * a_max)
    for i in range(1, samples + 1):
        f = dist * i / samples / sp
        if position_violation(p[0] + v[0] * f, p[1] + v[1] * f, p[2] + v[2] * f,
                              limits) is not None:
            return False
    return True


def envelope_step(current: Pose, velocity: Velocity, target: Pose, dt: float,
                  config: ArmBridgeConfig) -> tuple[Pose, Velocity]:
    """`accel_step`, kept inside the position envelope by braking, never by
    an instant reversal. The session validates TARGETS only; the commanded
    path between two valid targets can still cut a keep-out or the reach
    sphere, and a ramp can overshoot. Invariant: every commanded point and
    its braking path stay inside. If the ramped step would break it, the
    step brakes at the acceleration limit along the current direction
    instead -- whose path was checked last tick -- so the arm decelerates to
    rest at the boundary (a soft wall) and moves on once the target allows.
    A `current` that is already outside (re-seeded from a sagging arm) is
    exempt: the ramp may lead it back in."""
    ws = config.workspace
    a_lin = config.safety_factor * config.velocity.max_linear_acc_mm_s2
    pose, vel = accel_step(current, velocity, target, dt, config)
    if position_violation(current[0], current[1], current[2], ws) is not None:
        return pose, vel
    if _stop_path_clear(pose[:3], vel[0], a_lin, ws):
        return pose, vel
    v_prev = velocity[0]
    sp = _norm(v_prev)
    k = max(0.0, sp - a_lin * dt) / sp if sp > 0.0 else 0.0
    lin_v = tuple(c * k for c in v_prev)
    p = tuple(current[i] + lin_v[i] * dt for i in range(3))
    if position_violation(p[0], p[1], p[2], ws) is not None:
        lin_v, p = (0.0, 0.0, 0.0), tuple(current[:3])     # last resort: stay put
    return (p[0], p[1], p[2], pose[3], pose[4], pose[5]), (lin_v, vel[1])


def step_velocity(current: Pose, commanded: Pose, dt: float) -> Velocity:
    """The velocity a commanded step actually implies (after any downstream
    scaling, e.g. a per-joint speed cap), to carry into the next accel_step.
    Angular part: the base-frame rotation vector of the step / dt."""
    lin = tuple((commanded[i] - current[i]) / dt for i in range(3))
    ang = tuple(w / dt for w in rotation.rotvec_between(current[3:6], commanded[3:6]))
    return (lin, ang)


def speed(velocity: Velocity) -> tuple[float, float]:
    """(linear mm/s, angular deg/s) magnitudes of a Velocity."""
    return _norm(velocity[0]), _norm(velocity[1])


def clamp_gripper(current: float, target: float, dt: float, config: ArmBridgeConfig) -> float:
    """Clamp the gripper target into the hardware range, then rate-limit the step
    (rev 3 section 4: clamp range, throttle, ignore impossible values)."""
    lo, hi = config.workspace.gripper_min, config.workspace.gripper_max
    target = min(max(target, lo), hi)
    max_step = config.safety_factor * config.velocity.max_gripper_mm_s * dt
    delta = target - current
    if abs(delta) > max_step:
        delta = math.copysign(max_step, delta)
    return min(max(current + delta, lo), hi)
