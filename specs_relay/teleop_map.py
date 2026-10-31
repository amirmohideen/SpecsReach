"""Spectacles hand -> arm TCP target: the clutch, gates and mimicry mapping.

Pure logic, stdlib only (plus the bridge's own `rotation.py`), so it is unit
tested without GStreamer, a bridge or the glasses. `specs_relay.py` feeds it
the Lens samples and the bridge STATUS, and asks it once per heartbeat tick
what to send.

The control model is the Quest's (WORKING_QUEST_TELEOP.md §1, §4):

- The **clutch** is a pinch on the *clutch hand* (default: left). A press
  edge engages, provided STATUS is receipt-fresh, reads SERVO, carries no
  fault and no latched hold, and the *drive hand* (default: right) is
  tracked. At that instant the drive hand's wrist pose, the arm's actual TCP
  pose and the frame basis are recorded.
- While engaged the TCP moves and turns **as the wrist does**: translation
  one to one (times `position_scale`), rotation one to one about the TCP,
  both expressed through one basis change M from the Lens world to the arm
  base. A spatial rotation delta is pre-multiplied (the DROID / Quest
  "rigid attachment").
- Releasing the pinch, losing either hand, a stale Lens stream or any gate
  drops the deadman; the bridge stops the arm smoothly. Re-engaging needs a
  fresh pinch.

The wrist joint is the natural pivot (the Quest had to synthesise one 9 cm
behind the grip), so a pure wrist rotation reads as no translation.

Frames:

- Lens world W: centimetres, right-handed, +Y up, the camera looks along -Z.
- Arm base A: millimetres, right-handed, +Z up; orientation as RPY degrees,
  R = Rz(yaw) Ry(pitch) Rx(roll) (rotation.py).

M maps the operator triad (forward f, right r = f x up, up) in W onto the
arm triad (fa, ra = fa x z, z) in A. Both triads are built the same way in
right-handed spaces, so M is a proper rotation and R_A = M R_W M^T.
- "view" mode: f is the horizontal look direction at engage, and fa is the
  arm-base direction at `view_yaw_deg` -- the way the operator faces the arm
  when standing in their usual spot (0 = looking along arm +X).
- "room" mode: f is the calibrated world direction of arm +X (two wrist
  points, `calibrate_from_points`), and fa = arm +X. Survives turning the
  head, but the Lens world restarts with every Lens launch.
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

# The arm bridge's `arm_bridge` package: SpecsReach/arm_bridge/ next to this
# folder, or teleop-server/arm_bridge/ when the relay sits in its tools/.
_HERE = os.path.dirname(os.path.abspath(__file__))
for _root in (os.path.join(_HERE, "..", "..", "arm_bridge"), os.path.join(_HERE, "..", "arm_bridge")):
    if os.path.isdir(os.path.join(_root, "arm_bridge")):
        sys.path.insert(0, _root)
from arm_bridge import rotation  # noqa: E402
from arm_bridge.protocol import ArmState, Status  # noqa: E402

Vec3 = tuple[float, float, float]
Matrix = rotation.Matrix

IDENTITY: Matrix = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
WORLD_UP: Vec3 = (0.0, 1.0, 0.0)


# ---- small vector helpers ---------------------------------------------------
def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a: Vec3) -> float:
    return math.sqrt(_dot(a, a))


def _unit(a: Vec3) -> Vec3 | None:
    n = _norm(a)
    return None if n < 1e-9 else _scale(a, 1.0 / n)


def _mat_vec(m: Matrix, v: Vec3) -> Vec3:
    return (_dot(m[0], v), _dot(m[1], v), _dot(m[2], v))


def _mul(a: Matrix, b: Matrix) -> Matrix:
    return rotation._mul(a, b)


def _transpose(a: Matrix) -> Matrix:
    return rotation._transpose(a)


def _columns(x: Vec3, y: Vec3, z: Vec3) -> Matrix:
    return ((x[0], y[0], z[0]), (x[1], y[1], z[1]), (x[2], y[2], z[2]))


def _outer_sum(pairs) -> Matrix:
    """sum of a b^T over (a, b) pairs."""
    return tuple(tuple(sum(a[i] * b[j] for a, b in pairs) for j in range(3))
                 for i in range(3))  # type: ignore[return-value]


def _deadzone(v: Vec3, dz: float) -> Vec3:
    """Radial deadzone that subtracts, so leaving it has no jump."""
    n = _norm(v)
    if n <= dz:
        return (0.0, 0.0, 0.0)
    return _scale(v, (n - dz) / n)


def _clamp_norm(v: Vec3, limit: float) -> Vec3:
    n = _norm(v)
    return v if n <= limit or n < 1e-12 else _scale(v, limit / n)


# ---- frames -------------------------------------------------------------------
def horizontal(v: Vec3) -> Vec3 | None:
    """Unit projection of v onto the world's horizontal plane (None if ~vertical)."""
    return _unit((v[0], 0.0, v[2]))


def basis(world_forward: Vec3, arm_yaw_deg: float) -> Matrix:
    """M: Lens-world vectors -> arm-base vectors. `world_forward` (horizontal,
    unit) lands on the arm-base direction at `arm_yaw_deg` about +Z."""
    f = world_forward
    r = _cross(f, WORLD_UP)
    phi = math.radians(arm_yaw_deg)
    fa = (math.cos(phi), math.sin(phi), 0.0)
    za = (0.0, 0.0, 1.0)
    ra = _cross(fa, za)
    return _outer_sum(((fa, f), (ra, r), (za, WORLD_UP)))


def hand_rotation(wrist: Vec3, index_k: Vec3, middle_k: Vec3, pinky_k: Vec3) -> Matrix | None:
    """World rotation of the palm: x down the palm (wrist -> middle knuckle),
    z = x cross (pinky -> index knuckle), y = z cross x. Only deltas of it are
    used, so the choice of axes does not matter -- only that it is rigid."""
    x = _unit(_sub(middle_k, wrist))
    side = _unit(_sub(index_k, pinky_k))
    if x is None or side is None:
        return None
    z = _unit(_cross(x, side))
    if z is None:
        return None
    y = _cross(z, x)
    return _columns(x, y, z)


def yaw_from_points(start: Vec3, end: Vec3, min_cm: float) -> Vec3 | None:
    """Calibration: world direction of arm +X from two wrist points (the
    horizontal part only, so a height change does not matter). None if the
    move is shorter than `min_cm`."""
    d = _sub(end, start)
    d = (d[0], 0.0, d[2])
    if _norm(d) < min_cm:
        return None
    return _unit(d)


def _slerp_matrix(a: Matrix, b: Matrix, t: float) -> Matrix:
    """a moved fraction t of the way to b along the geodesic."""
    w = rotation.matrix_log(_mul(b, _transpose(a)))
    return _mul(rotation.matrix_exp(_scale(w, t)), a)


# ---- inputs -------------------------------------------------------------------
@dataclass(frozen=True)
class LensSample:
    """One Lens message (cm, Lens world). `drive_*` is the hand that moves
    the arm, `clutch_*` the hand whose pinch engages."""
    drive_tracked: bool
    wrist: Vec3 = (0.0, 0.0, 0.0)
    index_k: Vec3 = (0.0, 0.0, 0.0)
    middle_k: Vec3 = (0.0, 0.0, 0.0)
    pinky_k: Vec3 = (0.0, 0.0, 0.0)
    clutch_tracked: bool = False
    pinch: bool = False
    head_fwd: Vec3 | None = None   # look direction, world

    @staticmethod
    def from_json(msg: dict) -> "LensSample":
        """Parse a Lens `hand` message; raises ValueError on a malformed one."""
        def vec(v) -> Vec3:
            if not isinstance(v, (list, tuple)) or len(v) != 3:
                raise ValueError(f"bad vector {v!r}")
            out = tuple(float(c) for c in v)
            if not all(math.isfinite(c) for c in out):
                raise ValueError("non-finite vector")
            return out  # type: ignore[return-value]

        drive = msg.get("drive") or {}
        clutch = msg.get("clutch") or {}
        head = msg.get("head") or {}
        tracked = bool(drive.get("tracked"))
        kw = {}
        if tracked:
            kw = dict(wrist=vec(drive["wrist"]), index_k=vec(drive["index"]),
                      middle_k=vec(drive["middle"]), pinky_k=vec(drive["pinky"]))
        fwd = vec(head["fwd"]) if "fwd" in head else None
        return LensSample(drive_tracked=tracked, clutch_tracked=bool(clutch.get("tracked")),
                          pinch=bool(clutch.get("pinch")), head_fwd=fwd, **kw)


@dataclass
class Tuning:
    """Relay-side mapping knobs (specs_relay.json). Defaults follow the Quest
    (WORKING_QUEST_TELEOP.md §7), loosened where bare-hand tracking is
    noisier than a controller."""
    mode: str = "view"                 # "view" | "room"
    view_yaw_deg: float = 0.0          # arm-base yaw the operator faces in view mode
    position_scale: float = 1.0
    filter_tau_s: float = 0.06         # low-pass on the wrist pose
    deadzone_cm: float = 1.5           # radial, subtractive
    max_range_mm: float = 500.0        # box around the anchor, per axis
    max_speed_mm_s: float = 250.0      # glitch armour; the bridge ramps below it
    enable_orientation: bool = True
    orient_deadzone_deg: float = 8.0
    max_orient_deg: float = 120.0      # geodesic cone around the anchor
    max_orient_speed_deg_s: float = 90.0
    lens_timeout_s: float = 0.3        # no Lens message for this long = released
                                       # (the bridge's own watchdog is 300 ms)
    clutch_release_s: float = 0.15     # pinch must stay open this long to release:
                                       # hand tracking drops a held pinch for a frame or two
    status_fresh_s: float = 0.15       # STATUS older than this = refuse to drive
    calibration_min_cm: float = 20.0
    twin_orientation: bool = True      # digital twin: follow the model's tool rotation too
                                       # (its targets are noise-free, unlike a tracked hand)
    twin_max_speed_mm_s: float = 60.0      # digital twin: how fast the arm chases the model.
    twin_max_orient_speed_deg_s: float = 20.0  # A small model turns small hand moves into big
                                               # arm moves; the arm catches up at this pace.

    @staticmethod
    def from_dict(d: dict) -> "Tuning":
        t = Tuning()
        for key, value in d.items():
            if key.startswith("_"):
                continue
            if not hasattr(t, key):
                raise ValueError(f"unknown tuning key '{key}'")
            current = getattr(t, key)
            setattr(t, key, type(current)(value))
        if t.mode not in ("view", "room"):
            raise ValueError("mode must be 'view' or 'room'")
        return t


@dataclass(frozen=True)
class TwinSample:
    """One Lens `twin` message: the digital twin's TCP target, already in the
    arm base frame (mm, RPY degrees as the bridge uses them). `active` is the
    twin's clutch -- true while the operator holds a part of the model."""
    active: bool
    pos: Vec3
    rot: Matrix

    @staticmethod
    def from_json(msg: dict) -> "TwinSample":
        """Parse a Lens `twin` message; raises ValueError on a malformed one."""
        pose = msg.get("pose")
        if not isinstance(pose, (list, tuple)) or len(pose) != 6:
            raise ValueError(f"bad twin pose {pose!r}")
        v = tuple(float(c) for c in pose)
        if not all(math.isfinite(c) for c in v):
            raise ValueError("non-finite twin pose")
        return TwinSample(active=bool(msg.get("active")), pos=v[:3],
                          rot=rotation.rpy_to_matrix(*v[3:]))


@dataclass
class _TwinEngagement:
    twin_pos: Vec3          # twin target at engage, mm
    twin_rot: Matrix
    arm_pos: Vec3           # actual arm at engage, mm
    arm_rot: Matrix
    last_pos: Vec3
    last_rot: Matrix
    last_rpy: Vec3


@dataclass(frozen=True)
class Command:
    """What to put in the next heartbeat."""
    deadman: bool
    pose: tuple[float, float, float, float, float, float]
    gripper: float
    permit: int


@dataclass
class _Engagement:
    hand_pos: Vec3          # filtered wrist at engage, cm
    hand_rot: Matrix
    arm_pos: Vec3           # mm
    arm_rot: Matrix
    arm_rpy: Vec3
    m: Matrix               # frozen basis
    filt_pos: Vec3 = (0.0, 0.0, 0.0)
    filt_rot: Matrix = IDENTITY
    last_pos: Vec3 = (0.0, 0.0, 0.0)
    last_rot: Matrix = IDENTITY
    last_rpy: Vec3 = (0.0, 0.0, 0.0)


@dataclass
class Teleop:
    tuning: Tuning = field(default_factory=Tuning)
    room_forward: Vec3 | None = None       # calibrated world direction of arm +X

    _sample: LensSample | None = None
    _sample_t: float = -1e9
    _status: Status | None = None
    _status_t: float = -1e9
    _prev_clutch: bool | None = None
    _open_since: float | None = None       # engaged, but the pinch has been open since
    _eng: _Engagement | None = None
    _twin: TwinSample | None = None
    _twin_t: float = -1e9
    _twin_prev: bool | None = None
    _twin_eng: _TwinEngagement | None = None
    _last_tick: float | None = None
    note: str = "waiting for the Lens"

    # ---- inputs --------------------------------------------------------
    def on_lens(self, sample: LensSample, now: float) -> None:
        self._sample, self._sample_t = sample, now

    def on_twin(self, sample: TwinSample, now: float) -> None:
        self._twin, self._twin_t = sample, now

    def on_status(self, status: Status, now: float) -> None:
        self._status, self._status_t = status, now

    def forget_lens(self) -> None:
        self._sample = None
        self._twin = None

    @property
    def engaged(self) -> bool:
        return self._eng is not None or self._twin_eng is not None

    @property
    def status(self) -> Status | None:
        return self._status

    def latest_wrist(self, now: float) -> Vec3 | None:
        s = self._sample
        if s is None or not s.drive_tracked or now - self._sample_t > self.tuning.lens_timeout_s:
            return None
        return s.wrist

    def calibrate_from_points(self, start: Vec3, end: Vec3) -> bool:
        f = yaw_from_points(start, end, self.tuning.calibration_min_cm)
        if f is None:
            return False
        self.room_forward = f
        self.tuning.mode = "room"
        return True

    # ---- the per-heartbeat decision -------------------------------------
    def tick(self, now: float) -> Command | None:
        """The heartbeat to send now, or None when there is nothing valid to
        send yet (no STATUS pose seen)."""
        dt = 0.02 if self._last_tick is None else min(max(now - self._last_tick, 1e-3), 0.1)
        self._last_tick = now
        st = self._status
        if st is None:
            self.note = "no STATUS from the bridge yet"
            return None
        if self.note in ("no STATUS from the bridge yet", "waiting for the Lens"):
            self.note = "ready"
        t = self.tuning
        # The digital twin owns the arm while its messages flow (or it holds a
        # grab); the hand path then lets go and needs a fresh pinch afterwards.
        if self._twin_eng is not None or (self._twin is not None
                                          and now - self._twin_t <= t.lens_timeout_s):
            if self._eng is not None:
                self._eng = None
                self.note = "released: digital twin took over"
            self._prev_clutch = None
            self._open_since = None
            return self._tick_twin(now, dt, st)
        s = self._sample
        lens_ok = s is not None and now - self._sample_t <= t.lens_timeout_s
        clutch = lens_ok and s.clutch_tracked and s.pinch
        # A press is an observed released -> pinched edge. Without a live,
        # tracked, released sample before it (startup, Lens reconnect, clutch
        # hand coming into view already pinching) it is not a press.
        press = clutch and self._prev_clutch is False
        self._prev_clutch = clutch if lens_ok and s.clutch_tracked else None
        drive_ok = lens_ok and s.drive_tracked
        gate = self._gate(st, now)

        if self._eng is not None:
            # A pinch that opens only briefly is tracking flicker, not a release;
            # a silent Lens, a lost drive hand or a failed gate release at once.
            if clutch:
                self._open_since = None
            elif self._open_since is None:
                self._open_since = now
            pinch_gone = (not lens_ok or (not clutch and
                                          now - self._open_since >= t.clutch_release_s))
            why = ("Lens silent" if not lens_ok else
                   "pinch released" if pinch_gone else
                   "drive hand lost" if not drive_ok else gate)
            if why:
                self._eng = None
                self._open_since = None
                self.note = f"released: {why}"
        elif press:
            why = ("drive hand not tracked" if not drive_ok else gate or
                   ("room mode needs calibration" if t.mode == "room" and self.room_forward is None
                    else ""))
            if why:
                self.note = f"engage refused: {why}"
            elif not self._engage(s, st):
                self.note = "engage refused: hand pose degenerate"
            else:
                self.note = "engaged"

        if self._eng is None:
            # Idle heartbeats carry the actual pose so the bridge never has
            # to reject them (WORKING_QUEST_TELEOP.md §4.4).
            return Command(False, (st.x, st.y, st.z, st.roll, st.pitch, st.yaw),
                           st.gripper, st.motion_permit_id)
        pose = self._drive(s, dt)
        if pose is None:     # degenerate frame this tick: hold the last target
            pose = self._eng.last_pos + self._eng.last_rpy
        return Command(True, pose, st.gripper, st.motion_permit_id)

    def _tick_twin(self, now: float, dt: float, st: Status) -> Command:
        """Digital twin: the same clutch rules as the hand (press edge, gates,
        release), but the target is the model's TCP, already in the arm frame.
        The arm follows the target's motion since the grab (relative, like the
        hand), so a model drawn slightly off the actual pose cannot jump it."""
        t = self.tuning
        tw = self._twin
        live = tw is not None and now - self._twin_t <= t.lens_timeout_s
        active = live and tw.active
        press = active and self._twin_prev is False
        self._twin_prev = tw.active if live else None
        gate = self._gate(st, now)

        if self._twin_eng is not None:
            why = ("twin silent" if not live else
                   "twin let go" if not tw.active else gate)
            if why:
                self._twin_eng = None
                self.note = f"released: {why}"
        elif press:
            if gate:
                self.note = f"engage refused: {gate}"
            else:
                arm_pos = (st.x, st.y, st.z)
                arm_rot = rotation.rpy_to_matrix(st.roll, st.pitch, st.yaw)
                self._twin_eng = _TwinEngagement(
                    twin_pos=tw.pos, twin_rot=tw.rot, arm_pos=arm_pos, arm_rot=arm_rot,
                    last_pos=arm_pos, last_rot=arm_rot, last_rpy=(st.roll, st.pitch, st.yaw))
                self.note = "engaged (twin)"

        if self._twin_eng is None:
            return Command(False, (st.x, st.y, st.z, st.roll, st.pitch, st.yaw),
                           st.gripper, st.motion_permit_id)
        e = self._twin_eng
        d = _sub(tw.pos, e.twin_pos)
        lim = t.max_range_mm
        d = tuple(min(max(c, -lim), lim) for c in d)
        step = _clamp_norm(_sub(_add(e.arm_pos, d), e.last_pos), t.twin_max_speed_mm_s * dt)
        pos = _add(e.last_pos, step)
        if t.twin_orientation:
            w = _clamp_norm(rotation.matrix_log(_mul(tw.rot, _transpose(e.twin_rot))),
                            t.max_orient_deg)
            target_rot = _mul(rotation.matrix_exp(w), e.arm_rot)
            dw = _clamp_norm(rotation.matrix_log(_mul(target_rot, _transpose(e.last_rot))),
                             t.twin_max_orient_speed_deg_s * dt)
            rot_cmd = _mul(rotation.matrix_exp(dw), e.last_rot)
        else:
            rot_cmd = e.arm_rot
        rpy = rotation.matrix_to_rpy(rot_cmd)    # principal: the bridge's RPY box
        e.last_pos, e.last_rot, e.last_rpy = pos, rot_cmd, rpy
        return Command(True, pos + rpy, st.gripper, st.motion_permit_id)

    def _gate(self, st: Status, now: float) -> str:
        if now - self._status_t > self.tuning.status_fresh_s:
            return "STATUS stale"
        if st.arm_state != ArmState.SERVO:
            return f"arm is {st.arm_state.name}"
        if st.error_code:
            return f"arm error {st.error_code}"
        if st.hold_latched:
            return "server hold latched"
        return ""

    def _engage(self, s: LensSample, st: Status) -> bool:
        rot = hand_rotation(s.wrist, s.index_k, s.middle_k, s.pinky_k)
        if rot is None:
            return False
        if self.tuning.mode == "room":
            fwd, yaw = self.room_forward, 0.0
        else:
            fwd = horizontal(s.head_fwd) if s.head_fwd is not None else None
            if fwd is None:
                return False
            yaw = self.tuning.view_yaw_deg
        arm_rpy = (st.roll, st.pitch, st.yaw)
        arm_rot = rotation.rpy_to_matrix(*arm_rpy)
        arm_pos = (st.x, st.y, st.z)
        self._eng = _Engagement(hand_pos=s.wrist, hand_rot=rot, arm_pos=arm_pos,
                                arm_rot=arm_rot, arm_rpy=arm_rpy, m=basis(fwd, yaw),
                                filt_pos=s.wrist, filt_rot=rot,
                                last_pos=arm_pos, last_rot=arm_rot, last_rpy=arm_rpy)
        return True

    def _drive(self, s: LensSample, dt: float):
        e, t = self._eng, self.tuning
        rot = hand_rotation(s.wrist, s.index_k, s.middle_k, s.pinky_k)
        if rot is None:
            return None
        alpha = 1.0 - math.exp(-dt / t.filter_tau_s) if t.filter_tau_s > 0 else 1.0
        e.filt_pos = _add(e.filt_pos, _scale(_sub(s.wrist, e.filt_pos), alpha))
        e.filt_rot = _slerp_matrix(e.filt_rot, rot, alpha)

        # Translation: world cm -> arm mm through the frozen basis.
        d = _deadzone(_sub(e.filt_pos, e.hand_pos), t.deadzone_cm)
        d_arm = _scale(_mat_vec(e.m, d), 10.0 * t.position_scale)
        lim = t.max_range_mm
        d_arm = tuple(min(max(c, -lim), lim) for c in d_arm)
        target = _add(e.arm_pos, d_arm)
        step = _clamp_norm(_sub(target, e.last_pos), t.max_speed_mm_s * dt)
        pos = _add(e.last_pos, step)

        # Rotation: spatial delta, conjugated into the arm frame, about the TCP.
        if t.enable_orientation:
            w = rotation.matrix_log(_mul(e.filt_rot, _transpose(e.hand_rot)))
            w = _deadzone(w, t.orient_deadzone_deg)
            w = _clamp_norm(_mat_vec(e.m, w), t.max_orient_deg)  # M R M^T = exp(M w)
            target_rot = _mul(rotation.matrix_exp(w), e.arm_rot)
            dw = rotation.matrix_log(_mul(target_rot, _transpose(e.last_rot)))
            dw = _clamp_norm(dw, t.max_orient_speed_deg_s * dt)
            rot_cmd = _mul(rotation.matrix_exp(dw), e.last_rot)
        else:
            rot_cmd = e.arm_rot
        # The wire carries the principal RPY, each in (-180, 180] like the Quest
        # sends: the bridge rejects a target outside its roll/pitch/yaw box, and a
        # tool-down pose sits at roll +-180, so an unwrapped roll of -181 would
        # freeze the arm. Continuity is the bridge's job (rotation.py).
        rpy = rotation.matrix_to_rpy(rot_cmd)
        e.last_pos, e.last_rot, e.last_rpy = pos, rot_cmd, rpy
        return pos + rpy
