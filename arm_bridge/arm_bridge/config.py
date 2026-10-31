"""Arm bridge configuration: server-owned safety limits and loop rates.

Per rev 3 section 1, the server owns the hard safety backstop -- workspace,
orientation, velocity, and finite-value limits. These are NOT sent by the Quest;
they are configured here and enforced server-side regardless of what the client
asks for. Defaults are deliberately conservative for an xArm 850; tune per
deployment and re-review before widening.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields, replace

from .program_runtime import PROGRAM_FEATURES


class ConfigError(ValueError):
    """A configuration value is out of range; the bridge refuses to start."""


def validate_stun_server(value: str) -> None:
    """Validate the canonical STUN spelling: bare ``host:port``.

    One spelling only (e.g. ``stun.example.com:3478``); scheme'd forms
    (``stun:...`` / ``stun://...``) are rejected so configs never mix styles.
    The ``stun://`` prefix webrtcbin wants is added at the point of use
    (transport_webrtc). Raises ConfigError on any malformed value.
    """
    if "/" in value or value.startswith("stun:"):
        raise ConfigError(
            f"stun_server must be bare host:port, not a URL (got '{value}')")
    host, sep, port_text = value.rpartition(":")
    if not sep or not host:
        raise ConfigError(f"stun_server must be host:port (got '{value}')")
    try:
        port = int(port_text)
    except ValueError:
        raise ConfigError(f"stun_server port must be an integer (got '{port_text}')") from None
    if not 1 <= port <= 65535:
        raise ConfigError(f"stun_server port must be in [1, 65535] (got {port})")


def validate_bare_host(value: str, name: str) -> None:
    """Validate a robot controller address: a bare IPv4/hostname, no scheme,
    port, whitespace, or slash (ports are separate settings or fixed by the
    SDK, so a port here is always a mistake). Raises ConfigError."""
    if value != value.strip() or any(ch.isspace() for ch in value):
        raise ConfigError(f"{name} must not contain whitespace (got '{value}')")
    if "/" in value or "://" in value:
        raise ConfigError(f"{name} must be a bare host, not a URL (got '{value}')")
    if ":" in value:
        raise ConfigError(
            f"{name} must be a bare host without a port (got '{value}')")


def validate_xarm_ip(value: str) -> None:
    """Validate the xArm controller address (see validate_bare_host). The
    UFACTORY SDK connects by host only (its control ports are fixed). Empty is
    allowed at the config layer -- validate() requires it only for xarm."""
    validate_bare_host(value, "xarm_ip")


KEEPOUT_SHAPES = ("box", "cylinder", "sphere")


@dataclass(frozen=True)
class Keepout:
    """One keep-out zone in the arm base frame (mm). A commanded TCP inside it
    is rejected (the heartbeat is dropped and the arm holds at its last good
    target -- a soft wall, no fault), and the posture check refuses SET HOME /
    teleop engage while the ACTUAL TCP is inside. The check covers the TCP
    point only, never the elbow or links -- link collisions stay with the
    controller's safety configuration.

    shape: box (x/y/z ranges) | cylinder (axis parallel to z, centre cx/cy,
    radius, z range) | sphere (centre cx/cy/cz, radius). Unused keys stay 0.
    """
    shape: str = "cylinder"
    name: str = ""
    x_min: float = 0.0
    x_max: float = 0.0
    y_min: float = 0.0
    y_max: float = 0.0
    z_min: float = 0.0
    z_max: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    cz: float = 0.0
    radius: float = 0.0

    @property
    def label(self) -> str:
        return self.name or self.shape

    def validate(self, index: int) -> None:
        what = f"workspace.keepouts[{index}]"
        if self.shape not in KEEPOUT_SHAPES:
            raise ConfigError(f"{what}.shape must be one of {', '.join(KEEPOUT_SHAPES)} "
                              f"(got {self.shape!r})")
        if not isinstance(self.name, str):
            raise ConfigError(f"{what}.name must be a string")
        numeric = ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max",
                   "cx", "cy", "cz", "radius")
        for key in numeric:
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or value != value or value in (float("inf"), float("-inf")):
                raise ConfigError(f"{what}.{key} must be a finite number (got {value!r})")
        if self.shape == "box":
            for lo, hi, axis in ((self.x_min, self.x_max, "x"), (self.y_min, self.y_max, "y"),
                                 (self.z_min, self.z_max, "z")):
                if not lo < hi:
                    raise ConfigError(f"{what}: {axis}_min {lo} must be < {axis}_max {hi}")
        else:
            if not self.radius > 0:
                raise ConfigError(f"{what}: radius must be > 0 (got {self.radius})")
            if self.shape == "cylinder" and not self.z_min < self.z_max:
                raise ConfigError(f"{what}: z_min {self.z_min} must be < z_max {self.z_max}")

    def contains(self, x: float, y: float, z: float) -> bool:
        if self.shape == "box":
            return (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max
                    and self.z_min <= z <= self.z_max)
        if self.shape == "cylinder":
            dx, dy = x - self.cx, y - self.cy
            return (dx * dx + dy * dy <= self.radius * self.radius
                    and self.z_min <= z <= self.z_max)
        dx, dy, dz = x - self.cx, y - self.cy, z - self.cz
        return dx * dx + dy * dy + dz * dz <= self.radius * self.radius

    def to_json(self) -> dict:
        """The keys that matter for this shape (what the YAML / form / viewer carry)."""
        out: dict = {"shape": self.shape, "name": self.name}
        if self.shape == "box":
            out.update(x_min=self.x_min, x_max=self.x_max, y_min=self.y_min,
                       y_max=self.y_max, z_min=self.z_min, z_max=self.z_max)
        elif self.shape == "cylinder":
            out.update(cx=self.cx, cy=self.cy, radius=self.radius,
                       z_min=self.z_min, z_max=self.z_max)
        else:
            out.update(cx=self.cx, cy=self.cy, cz=self.cz, radius=self.radius)
        return out


def keepout_from_mapping(mapping, index: int) -> Keepout:
    if not isinstance(mapping, dict):
        raise ConfigError(f"workspace.keepouts[{index}] must be a mapping "
                          "({shape: cylinder, name: ..., radius: ..., ...})")
    allowed = {f.name for f in fields(Keepout)}
    unknown = sorted(set(mapping) - allowed)
    if unknown:
        raise ConfigError(f"unknown workspace.keepouts[{index}] key(s): {', '.join(unknown)}")
    return Keepout(**mapping)


@dataclass(frozen=True)
class WorkspaceLimits:
    """Hard Cartesian + orientation box, xArm base frame (mm / deg). A target
    outside this box is rejected outright (rev 3 section 4), never clamped, so a
    client bug cannot walk the arm to a boundary."""
    x_min: float = -700.0
    x_max: float = 700.0
    y_min: float = -700.0
    y_max: float = 700.0
    z_min: float = 0.0
    z_max: float = 900.0
    roll_min: float = -180.0
    roll_max: float = 180.0
    pitch_min: float = -180.0
    pitch_max: float = 180.0
    yaw_min: float = -180.0
    yaw_max: float = 180.0
    gripper_min: float = 2.0   # rev 3 section 2: clamp to [2, 84] unless hw differs
    gripper_max: float = 84.0
    # Reach backstop: the box above is a cuboid, but the arm's reachable space is
    # a sphere, so a box corner (e.g. x=y=700) can sit far outside the arm's reach
    # yet inside the box -- the controller then grinds the elbow into its joint
    # limit chasing it and faults (error 23, straight-elbow J3). Reject targets
    # whose TCP is farther than `reach_max_mm` from the shoulder centre
    # (0, 0, reach_center_z). This is a COARSE, orientation-independent outer
    # bound: the default clears the true max tip reach (UF850 flange ~850 mm +
    # 172 mm TCP ~= 1022 mm) so it never rejects a physically reachable pose, only
    # gross box-corner violations. The controller's own IK is the real per-
    # orientation limit. Set reach_max_mm: 0 to disable.
    reach_max_mm: float = 1050.0
    reach_center_z: float = 364.0   # UF850 shoulder (J2 axis) height, base frame
                                    # (URDF joint1 origin z=0.364 m)
    # Keep-out zones inside the box (see Keepout): a commanded TCP inside one
    # is rejected like an out-of-box target. Empty by default; the station
    # config fences its base column. YAML: a one-line flow list of mappings.
    keepouts: tuple = ()
    # Retreat margin: how far past a limit / zone boundary the RETREAT move
    # places the TCP (mm) or a joint (deg), so the arm lands clearly inside.
    retreat_margin_mm: float = 30.0
    retreat_margin_deg: float = 5.0

    def __post_init__(self) -> None:
        # YAML gives a list of mappings; normalise to Keepout objects once so
        # every consumer (safety, hello, form) sees one type. Non-mappings are
        # a ConfigError here rather than a crash deep in validate().
        if isinstance(self.keepouts, (list, tuple)):
            zones = tuple(z if isinstance(z, Keepout) else keepout_from_mapping(z, i)
                          for i, z in enumerate(self.keepouts))
            object.__setattr__(self, "keepouts", zones)

    def validate(self) -> None:
        for lo, hi, name in (
            (self.x_min, self.x_max, "x"), (self.y_min, self.y_max, "y"),
            (self.z_min, self.z_max, "z"), (self.roll_min, self.roll_max, "roll"),
            (self.pitch_min, self.pitch_max, "pitch"), (self.yaw_min, self.yaw_max, "yaw"),
            (self.gripper_min, self.gripper_max, "gripper"),
        ):
            if not lo < hi:
                raise ConfigError(f"workspace {name}: min {lo} must be < max {hi}")
        if self.reach_max_mm < 0:
            raise ConfigError(
                f"workspace reach_max_mm must be >= 0 (0 disables) (got {self.reach_max_mm})")
        if not isinstance(self.keepouts, (list, tuple)):
            raise ConfigError("workspace.keepouts must be a list of keep-out mappings")
        for i, zone in enumerate(self.keepouts):
            if not isinstance(zone, Keepout):
                raise ConfigError(f"workspace.keepouts[{i}] must be a keep-out mapping")
            zone.validate(i)
        if not self.retreat_margin_mm > 0 or not self.retreat_margin_deg > 0:
            raise ConfigError("workspace retreat_margin_mm / retreat_margin_deg must be > 0")


@dataclass(frozen=True)
class VelocityLimits:
    """Per-axis-group speed caps. The servo loop clamps each step's motion by
    vector norm so combined multi-axis motion stays at or below `safety_factor`
    of these (rev 3 section 4)."""
    max_linear_mm_s: float = 250.0
    max_angular_deg_s: float = 60.0   # gentler servo near wrist singularities/limits,
                                      # where a small Cartesian rotation needs large
                                      # joint velocity (a cause of servo error 35)
    max_gripper_mm_s: float = 100.0
    # Acceleration caps (x safety_factor), applied by backends that rate-limit
    # the stream themselves (AUBO; the xArm's mode-7 planner has its own).
    # Without them every target jump or reversal was an instant velocity step:
    # hand jitter at ~75 mm/s commanded ~7 m/s^2 jolts and tripped the AUBO's
    # torque-based collision detection (2026-10-12; a 10 mm/s ramp did not).
    max_linear_acc_mm_s2: float = 1000.0
    max_angular_acc_deg_s2: float = 300.0

    def validate(self) -> None:
        for v, name in ((self.max_linear_mm_s, "max_linear_mm_s"),
                        (self.max_angular_deg_s, "max_angular_deg_s"),
                        (self.max_gripper_mm_s, "max_gripper_mm_s"),
                        (self.max_linear_acc_mm_s2, "max_linear_acc_mm_s2"),
                        (self.max_angular_acc_deg_s2, "max_angular_acc_deg_s2")):
            if not v > 0:
                raise ConfigError(f"velocity {name} must be > 0 (got {v})")


ARM_BACKENDS = ("mock", "sim", "xarm", "aubo")

# xArm report types and the commissioning keys the operator must set explicitly
# in the YAML before a real arm may actuate (safety review #2).
_XARM_REPORT_TYPES = ("normal", "rich", "real")
_XARM_COMMISSION_KEYS = ("tcp_offset", "payload_kg", "payload_cog")


@dataclass(frozen=True)
class XArmTuning:
    """xArm-only controller tuning + commissioning values. Read ONLY when
    `arm_backend == 'xarm'`; the mock ignores it. Values marked (HW) must be
    confirmed/calibrated on the physical 850 -- this block is where that
    calibration lives, so tuning on hardware is a config edit, not a code change.
    The state-machine / collision-classification *logic* stays in code (it is a
    state machine, not a value); only the tunable values live here.
    """
    expected_model: str = ""        # required for xarm: verified vs arm.device_type
    expected_sn: str = ""           # optional: verified only when set
    report_type: str = "normal"     # (HW) SDK report stream; confirm achieved rate
                                    # vs report_stale_ms (review: 'rich' ~5 Hz)
    collision_codes: tuple = (22,)  # (HW) controller error codes -> PROTECTIVE_STOP
                                    # fallback; 22 = self-collision. Prefer the
                                    # safeguard/motor-enable state (in code).
    gripper_scale: float = 10.0     # (HW) wire mm -> gripper units (~0-850 span)
    gripper_offset: float = 0.0     # (HW) gripper unit offset
    gripper_speed: int = 5000       # gripper move speed (SDK units)
    gripper_force: int = 25         # (HW) G2 gripper force, newtons (10-50); low = safe
    servo_speed: float = 200.0      # set_servo_cartesian speed arg (reserved by
    servo_mvacc: float = 2000.0     # servo mode; kept for completeness)
    # Mode-7 (online trajectory planning) motion caps for set_position. The
    # CONTROLLER enforces these, so they -- not a client-side per-tick step --
    # are the drive-rate safety cap. Conservative defaults; raise on hardware.
    move_speed_mm_s: float = 120.0
    move_acc_mm_s2: float = 800.0
    tcp_offset: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # (HW) x,y,z,roll,pitch,yaw
    payload_kg: float = 0.0         # (HW) payload mass at the TCP
    payload_cog: tuple = (0.0, 0.0, 0.0)  # (HW) payload centre of gravity, mm
    report_stale_ms: float = 200.0  # SDK report older than this -> snapshot UNKNOWN

    @property
    def provided(self) -> frozenset:
        """Keys explicitly present in the YAML block (all, for code-built
        instances). Used to enforce that commissioning values are set, not
        defaulted -- set by `config_from_mapping`."""
        return getattr(self, "_provided",
                       frozenset(f.name for f in fields(XArmTuning)))

    def validate(self) -> None:
        if self.report_type not in _XARM_REPORT_TYPES:
            raise ConfigError(
                f"xarm.report_type must be one of {_XARM_REPORT_TYPES} "
                f"(got '{self.report_type}')")
        if any(int(c) < 0 for c in self.collision_codes):
            raise ConfigError("xarm.collision_codes must be non-negative integers")
        if self.gripper_scale == 0.0:
            raise ConfigError("xarm.gripper_scale must be non-zero")
        if not self.gripper_speed > 0:
            raise ConfigError(f"xarm.gripper_speed must be > 0 (got {self.gripper_speed})")
        if not self.report_stale_ms > 0:
            raise ConfigError(f"xarm.report_stale_ms must be > 0 (got {self.report_stale_ms})")
        if len(tuple(self.tcp_offset)) != 6:
            raise ConfigError("xarm.tcp_offset must be 6 values [x,y,z,roll,pitch,yaw]")
        if len(tuple(self.payload_cog)) != 3:
            raise ConfigError("xarm.payload_cog must be 3 values [x,y,z]")
        if not self.payload_kg >= 0:
            raise ConfigError(f"xarm.payload_kg must be >= 0 (got {self.payload_kg})")

    def require_commissioned(self) -> None:
        """Fail-closed gate before a real arm may actuate (Q2: model + values,
        serial optional). The operator must name the robot and explicitly set the
        TCP/payload commissioning values -- silent defaults are refused."""
        if not self.expected_model:
            raise ConfigError(
                "arm_backend 'xarm' requires xarm.expected_model (the robot to verify)")
        missing = [k for k in _XARM_COMMISSION_KEYS if k not in self.provided]
        if missing:
            raise ConfigError(
                "arm_backend 'xarm' commissioning: set "
                f"{', '.join(missing)} explicitly in the xarm config block")


# AUBO commissioning keys the operator must set explicitly (like the xArm's).
_AUBO_COMMISSION_KEYS = ("tcp_offset", "payload_kg", "payload_cog")
# Rate cap for AUBO streaming: servoJoint/trackJoint accept ~140 points/s at
# most and the lag grows from ~125 Hz (aubo-control-notes.md §6.3).
AUBO_SERVO_HZ_MAX = 100.0


@dataclass(frozen=True)
class AuboTuning:
    """AUBO (ARCS controller, pyaubo_sdk) connection, commissioning and
    teleop tuning. Read ONLY when `arm_backend == 'aubo'`. Values marked (HW)
    must be confirmed on the physical arm; see aubo-control-notes.md."""
    host: str = ""                  # controller address, bare host (required)
    rpc_port: int = 30004
    user: str = "aubo"              # ARCS factory credentials
    password: str = "123456"
    request_timeout_ms: int = 1000  # RPC request timeout
    state_hz: float = 100.0         # state poll rate (5 RPC reads, ~4 ms per poll)
    report_stale_ms: float = 100.0  # state older than this -> snapshot UNKNOWN
    # Commissioning (must be set explicitly -- silent defaults are refused).
    # Written to the controller at connect, so the config is the one source.
    tcp_offset: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)  # (HW) mm + deg, x,y,z,roll,pitch,yaw
    payload_kg: float = 0.0         # (HW) mass at the flange/TCP
    payload_cog: tuple = (0.0, 0.0, 0.0)                # (HW) centre of gravity, mm
    # Bridge-owned joint limits: the controller allows +/-360 deg on EVERY
    # joint, so these are the real envelope. IK solutions outside are refused
    # (the arm holds). J2/J3 per the i5 datasheet (+/-175). (HW) tighten.
    joint_min_deg: tuple = (-355.0, -175.0, -175.0, -355.0, -355.0, -355.0)
    joint_max_deg: tuple = (355.0, 175.0, 175.0, 355.0, 355.0, 355.0)
    # Teleop speed caps on top of the Cartesian velocity limits: no joint
    # moves faster than this (near singularities a small Cartesian step needs
    # a large joint step -- the arm lags instead of whipping).
    max_joint_speed_deg_s: float = 90.0
    # Per-joint acceleration cap while teleoperating (MIMICRY-TELEOP-PLAN.md
    # B4), absolute like the speed cap. Also brakes a joint so it stops ON its
    # bridge limit instead of being refused at speed. The i5H's own joint
    # acceleration limit is ~1490-1780 deg/s^2.
    max_joint_acc_deg_s2: float = 400.0
    # A joint jump larger than this in ONE tick is an IK branch flip
    # (singularity crossing): refused, the arm holds.
    ik_flip_deg: float = 30.0
    # Elbow guard: teleop never lets |J3| close below this (0 = off). A
    # straight elbow is singular -- the IK steps through J3 = 0 onto the
    # mirrored elbow, and just past full reach only a different posture solves
    # (163 deg IK jumps and protective stops at full stretch, 2026-10-26). The
    # arm brakes onto it on the commanded path (warn 1002). Costs ~7 mm of reach
    # on the i5H at 15 deg.
    elbow_min_deg: float = 15.0
    track_smooth_scale: float = 0.5  # trackJoint smoothing (no measured effect)
    track_delay_scale: float = 1.0   # trackJoint delay (no measured effect)
    stop_joint_acc_deg_s2: float = 300.0   # stopJoint deceleration on hold/fault
    # Smooth stop when a teleop burst ends while the arm moves (clutch release,
    # watchdog, hold) or a target is held mid-motion: one controller-planned
    # stopJoint at stop_joint_acc_deg_s2 instead of leaving the last queued
    # trackJoint point as an instant velocity step (that step tripped the
    # torque-based collision stop, 2026-10-12). Off = the old behaviour; the
    # switch exists until stopJoint-during-trackJoint is measured on the arm
    # (MIMICRY-TELEOP-PLAN.md B0).
    smooth_stop: bool = True
    smooth_stop_timeout_s: float = 2.0     # wait at most this long for "steady" after it
    home_speed_deg_s: float = 20.0   # HOME moveJoint speed (x pendant speed fraction)
    home_acc_deg_s2: float = 80.0
    power_timeout_s: float = 30.0    # poweron -> Idle and startup -> Running, each
    # Programs tab (Lua on the controller, script port 30002). Run and Stop
    # always work; each optional feature is switched on here only after it was
    # VERIFIED on this controller (aubo-control-notes.md §12): any of
    # line, pause, step, breakpoints, simulate, output.
    script_port: int = 30002
    program_features: tuple = ()
    stop_line_acc: float = 2.0       # stopLine on Stop: linear decel, m/s^2
    stop_line_acc_rot: float = 3.0   # stopLine on Stop: angular decel, rad/s^2

    @property
    def provided(self) -> frozenset:
        """Keys explicitly present in the YAML block (all, for code-built
        instances); set by `config_from_mapping`."""
        return getattr(self, "_provided",
                       frozenset(f.name for f in fields(AuboTuning)))

    def validate(self) -> None:
        if self.host:
            validate_bare_host(self.host, "aubo.host")
        if not (isinstance(self.rpc_port, int) and 1 <= self.rpc_port <= 65535):
            raise ConfigError(f"aubo.rpc_port must be 1-65535 (got {self.rpc_port!r})")
        if not self.request_timeout_ms > 0:
            raise ConfigError(f"aubo.request_timeout_ms must be > 0 (got {self.request_timeout_ms})")
        if not 0 < self.state_hz <= 200:
            raise ConfigError(f"aubo.state_hz must be in (0, 200] (got {self.state_hz})")
        if not self.report_stale_ms >= 2 * 1000.0 / self.state_hz:
            raise ConfigError(
                f"aubo.report_stale_ms ({self.report_stale_ms}) must cover at least "
                f"two state polls at state_hz {self.state_hz}")
        if len(tuple(self.tcp_offset)) != 6:
            raise ConfigError("aubo.tcp_offset must be 6 values [x,y,z,roll,pitch,yaw]")
        if len(tuple(self.payload_cog)) != 3:
            raise ConfigError("aubo.payload_cog must be 3 values [x,y,z]")
        if not self.payload_kg >= 0:
            raise ConfigError(f"aubo.payload_kg must be >= 0 (got {self.payload_kg})")
        lo, hi = tuple(self.joint_min_deg), tuple(self.joint_max_deg)
        if len(lo) != 6 or len(hi) != 6:
            raise ConfigError("aubo.joint_min_deg / joint_max_deg must be 6 values")
        for i, (a, b) in enumerate(zip(lo, hi)):
            if not -360.0 <= a < b <= 360.0:
                raise ConfigError(
                    f"aubo joint {i + 1} limits must satisfy -360 <= min < max <= 360 "
                    f"(got [{a}, {b}])")
        if not (isinstance(self.script_port, int) and 1 <= self.script_port <= 65535):
            raise ConfigError(f"aubo.script_port must be 1-65535 (got {self.script_port!r})")
        unknown = sorted(set(self.program_features) - set(PROGRAM_FEATURES))
        if unknown:
            raise ConfigError(f"aubo.program_features: unknown {', '.join(map(str, unknown))} "
                              f"(choose from {', '.join(PROGRAM_FEATURES)})")
        for value, name in ((self.max_joint_speed_deg_s, "max_joint_speed_deg_s"),
                            (self.stop_line_acc, "stop_line_acc"),
                            (self.stop_line_acc_rot, "stop_line_acc_rot"),
                            (self.ik_flip_deg, "ik_flip_deg"),
                            (self.stop_joint_acc_deg_s2, "stop_joint_acc_deg_s2"),
                            (self.smooth_stop_timeout_s, "smooth_stop_timeout_s"),
                            (self.max_joint_acc_deg_s2, "max_joint_acc_deg_s2"),
                            (self.home_speed_deg_s, "home_speed_deg_s"),
                            (self.home_acc_deg_s2, "home_acc_deg_s2"),
                            (self.power_timeout_s, "power_timeout_s")):
            if not value > 0:
                raise ConfigError(f"aubo.{name} must be > 0 (got {value})")
        if not 0.0 <= self.elbow_min_deg < 90.0:
            raise ConfigError(f"aubo.elbow_min_deg must be in [0, 90) (got {self.elbow_min_deg})")

    def require_commissioned(self) -> None:
        """Fail-closed gate before a real AUBO may actuate (mirrors the
        xArm's): the operator must name the controller and explicitly set the
        TCP/payload commissioning values."""
        if not self.host:
            raise ConfigError("arm_backend 'aubo' requires aubo.host (controller address)")
        missing = [k for k in _AUBO_COMMISSION_KEYS if k not in self.provided]
        if missing:
            raise ConfigError(
                "arm_backend 'aubo' commissioning: set "
                f"{', '.join(missing)} explicitly in the aubo config block")


@dataclass(frozen=True)
class ProgramsConfig:
    """Programs tab: Lua programs run on the controller (backends with
    supports_programs -- AUBO, and the mock's fake runtime)."""
    dir: str = "programs"            # program files; relative = to the repo root
    watchdog_s: float = 5.0          # stop a run when no dashboard watches this long; 0 = off
    speed_default: float = 0.25      # Run dialog default speed fraction
    speed_max: float = 1.0           # highest speed fraction the Run dialog offers
    history_keep: int = 200          # run records kept in <dir>/.runs

    def validate(self) -> None:
        if not isinstance(self.dir, str) or not self.dir.strip():
            raise ConfigError("programs.dir must be a non-empty path")
        if not self.watchdog_s >= 0:
            raise ConfigError(f"programs.watchdog_s must be >= 0 (got {self.watchdog_s})")
        if not 0.0 < self.speed_max <= 1.0:
            raise ConfigError(f"programs.speed_max must be in (0, 1] (got {self.speed_max})")
        if not 0.0 < self.speed_default <= self.speed_max:
            raise ConfigError(f"programs.speed_default must be in (0, speed_max] "
                              f"(got {self.speed_default})")
        if not (isinstance(self.history_keep, int) and self.history_keep >= 1):
            raise ConfigError(f"programs.history_keep must be >= 1 (got {self.history_keep!r})")


@dataclass(frozen=True)
class LinkQualityConfig:
    """Teleop link-quality grading for the dashboard (link_stats.py). Display
    and alerting only: none of these values gates motion -- the watchdog and
    the motion permit do that."""
    window_s: float = 5.0            # rolling window the readout summarizes
    rtt_fair_ms: float = 80.0        # loop RTT (median) at/above this is fair
    rtt_poor_ms: float = 150.0       # ... and at/above this, poor
    loss_fair_pct: float = 1.0       # heartbeat / video packet loss, fair
    loss_poor_pct: float = 5.0       # ... poor
    gap_fair_ms: float = 100.0       # longest heartbeat gap in the window, fair
    gap_poor_ms: float = 200.0       # ... poor (the deployed watchdog holds at 300)
    alert_after_s: float = 3.0       # poor this long -> the alert lane says so
    clear_after_s: float = 5.0       # not poor this long -> the alert clears

    def validate(self) -> None:
        if not 1.0 <= self.window_s <= 60.0:
            raise ConfigError(f"link_quality.window_s must be in [1, 60] (got {self.window_s})")
        for fair, poor, name in ((self.rtt_fair_ms, self.rtt_poor_ms, "rtt"),
                                 (self.loss_fair_pct, self.loss_poor_pct, "loss"),
                                 (self.gap_fair_ms, self.gap_poor_ms, "gap")):
            if not 0.0 < fair <= poor:
                raise ConfigError(f"link_quality: need 0 < {name} fair <= {name} poor "
                                  f"(got {fair}, {poor})")
        if not self.alert_after_s >= 0 or not self.clear_after_s >= 0:
            raise ConfigError("link_quality.alert_after_s / clear_after_s must be >= 0")


@dataclass(frozen=True)
class CaptureConfig:
    """Arm data capture (CAPTURE-SYNC-PLAN.md S3): hand timestamped arm
    records -- every heartbeat received, every servo tick's command, the
    measured state -- to the station's capture node over a local socket.
    Off by default; while on and nobody listens it costs one attribute read
    per record point. Never touches motion."""
    enabled: bool = False
    # Abstract AF_UNIX socket name (no leading NUL) the capture node binds.
    socket_name: str = "teleop-capture-arm"
    # Records held while the drain thread catches up; the oldest drop first.
    queue_max: int = 4096

    def validate(self) -> None:
        if not self.socket_name or "\0" in self.socket_name or len(self.socket_name) > 100:
            raise ConfigError("capture.socket_name must be 1-100 chars, no NUL")
        if not 64 <= self.queue_max <= 1_000_000:
            raise ConfigError(f"capture.queue_max must be in [64, 1000000] (got {self.queue_max})")


@dataclass(frozen=True)
class ArmBridgeConfig:
    # Which ArmController drives the arm. Config-only (no CLI): 'mock' keeps the
    # current loopback/test behavior unchanged; 'sim' runs the PyBullet UF850
    # kinematic simulator (no hardware, real kinematics -- SIM-DASHBOARD-PLAN);
    # 'xarm' actuates a real 850; 'aubo' an AUBO arm on an ARCS controller.
    arm_backend: str = "mock"
    # Which arm this is -- the ARM TYPE: a vendored model id or alias
    # (robot_models.py; dashboard/static/assets/<id>/), drawn by the dashboard
    # 3D viewer. Must be drivable by arm_backend (the model's manifest lists its
    # backends; the mock accepts any model, for previews). Empty = the
    # driver's default (mock/sim/xarm: uf850).
    robot_model: str = ""
    servo_hz: float = 250.0            # rev 3 section 4: steady servo loop, ~250 Hz
    status_hz: float = 30.0           # STATUS emission rate (rev 3 section 3: 20-50 Hz)
    watchdog_ms: float = 150.0        # rev 3 section 4: force hold after 150 ms silence
    # Command-freshness permit (TURN Phase 1 §5.6). Max server-clock age of the
    # motionPermitId echoed in a heartbeat before it holds. Intentionally LOOSE
    # for now (well above the 150 ms watchdog) so it never false-holds before we
    # have measured baseline data; tighten inside the watchdog once we do.
    motion_permit_max_age_ms: float = 2000.0
    safety_factor: float = 0.70       # rev 3 section 4: motion <= 70% of limits
    gripper_command_min_interval_s: float = 0.05  # throttle gripper commands
    # STUN server as bare "host:port" (WAN plan S1). Empty = LAN mode: webrtcbin
    # gathers host candidates only and everything behaves exactly as before.
    stun_server: str = ""
    # xArm controller address on the station LAN, bare host (no port -- the SDK
    # ports are fixed). Empty until set; required only when --arm xarm actuates
    # (Increment 3). Mock ignores it.
    xarm_ip: str = ""
    # Web dashboard (SIM-DASHBOARD-PLAN). Off unless a port is set. v1 is
    # read-only (observe + logs) and binds to loopback by default -- the
    # dashboard can edit safety limits in later phases and station_agent.yaml
    # holds a WAN secret, so exposure beyond localhost is an explicit operator
    # choice (SSH tunnel is the supported remote path).
    dashboard_port: int | None = None
    dashboard_bind: str = "127.0.0.1"
    # Live camera tabs in the dashboard. The streamer is a separate process that
    # holds the cameras and serves WebRTC signaling (default :8443/rtc); the
    # dashboard only advertises the endpoint the browser connects to, and the
    # browser talks to the streamer's /monitor path on that host:port (camera
    # list + Stream switch + its own views) -- the bridge stays out of the media.
    # Off by default -- the operator enables it when the streamer runs with
    # --transport webrtc. camera_signaling empty = browser derives
    # ws://<dashboard host>:8443/rtc; set it to override the host/port.
    camera_view: bool = False
    camera_signaling: str = ""
    # Motion journal (MIMICRY-TELEOP-PLAN.md B5): the last journal_seconds of
    # servo ticks (targets, commands, actual pose/joints, codes) kept in
    # memory, written as JSONL to journal_dir on a fault / protective stop and
    # on demand (dashboard POST /api/journal/dump). 0 disables.
    journal_seconds: float = 10.0
    journal_dir: str = "/data/captures/journal"
    # The protocol version the HELLO reply negotiates (protocol.py). 4 asks the
    # Quest for rev 0x04 heartbeats (pose time + shown-frame capture times).
    # Today's Quest accepts exactly 3: switch to 4 only after every headset
    # runs a build that accepts both (CAPTURE-SYNC-QUEST-HANDOFF.md §2).
    protocol_version: int = 3
    workspace: WorkspaceLimits = field(default_factory=WorkspaceLimits)
    velocity: VelocityLimits = field(default_factory=VelocityLimits)
    xarm: XArmTuning = field(default_factory=XArmTuning)
    aubo: AuboTuning = field(default_factory=AuboTuning)
    programs: ProgramsConfig = field(default_factory=ProgramsConfig)
    link_quality: LinkQualityConfig = field(default_factory=LinkQualityConfig)
    capture: CaptureConfig = field(default_factory=CaptureConfig)

    # UFACTORY set_servo_cartesian constraints (safety review): 30-250 Hz receive
    # window, and each commanded linear step must stay under this bound to avoid a
    # C1 "speed exceeds limit" fault. The step clamp already respects it at sane
    # configs; validate() makes it a hard gate instead of a hope.
    # Unannotated on purpose: annotating would make @dataclass treat these as
    # fields. Plain class attributes stay out of the field set / constructor.
    SERVO_HZ_MIN = 30.0
    SERVO_HZ_MAX = 250.0
    MAX_LINEAR_STEP_MM = 10.0

    def validate(self) -> None:
        if self.arm_backend not in ARM_BACKENDS:
            raise ConfigError(
                f"arm_backend must be one of {', '.join(ARM_BACKENDS)} "
                f"(got '{self.arm_backend}')")
        if self.arm_backend == "xarm":
            if not self.xarm_ip:
                raise ConfigError("arm_backend 'xarm' requires xarm_ip (bare host)")
            self.xarm.require_commissioned()
        if self.arm_backend == "aubo":
            self.aubo.require_commissioned()
            # One AUBO driver drives every ARCS model, so the station must name
            # the one it expects; connect() verifies the robot against it.
            if not self.robot_model:
                raise ConfigError(
                    "arm_backend 'aubo' requires robot_model (e.g. aubo_i5H_10)")
            if self.servo_hz > AUBO_SERVO_HZ_MAX:
                raise ConfigError(
                    f"arm_backend 'aubo' streams at most {AUBO_SERVO_HZ_MAX:.0f} Hz "
                    f"(controller drops points above ~140 Hz); got servo_hz {self.servo_hz}")
        if not self.servo_hz > 0:
            raise ConfigError(f"servo_hz must be > 0 (got {self.servo_hz})")
        if not self.SERVO_HZ_MIN <= self.servo_hz <= self.SERVO_HZ_MAX:
            raise ConfigError(
                f"servo_hz must be in [{self.SERVO_HZ_MIN:.0f}, {self.SERVO_HZ_MAX:.0f}] "
                f"for set_servo_cartesian (got {self.servo_hz})")
        if not self.status_hz > 0:
            raise ConfigError(f"status_hz must be > 0 (got {self.status_hz})")
        if not 0.0 <= self.journal_seconds <= 120.0:
            raise ConfigError(f"journal_seconds must be in [0, 120] (got {self.journal_seconds})")
        if self.journal_seconds > 0 and not self.journal_dir:
            raise ConfigError("journal_dir must be set when journal_seconds > 0")
        if not self.watchdog_ms > 0:
            raise ConfigError(f"watchdog_ms must be > 0 (got {self.watchdog_ms})")
        if not 0.0 < self.safety_factor <= 1.0:
            raise ConfigError(f"safety_factor must be in (0, 1] (got {self.safety_factor})")
        if self.stun_server:
            validate_stun_server(self.stun_server)
        if self.xarm_ip:
            validate_xarm_ip(self.xarm_ip)
        if self.dashboard_port is not None:
            if not (isinstance(self.dashboard_port, int)
                    and 1 <= self.dashboard_port <= 65535):
                raise ConfigError(
                    f"dashboard_port must be 1-65535 (got {self.dashboard_port!r})")
            if not self.dashboard_bind:
                raise ConfigError("dashboard_bind must be non-empty when "
                                  "dashboard_port is set")
        if not isinstance(self.robot_model, str):
            raise ConfigError(f"robot_model must be a string (got {self.robot_model!r})")
        if self.robot_model:
            from .robot_models import default_catalog   # stdlib-only, scanned once
            catalog = default_catalog()
            model = catalog.resolve(self.robot_model)
            if model is None:
                raise ConfigError(
                    f"robot_model '{self.robot_model}' is not a vendored model "
                    f"(known: {', '.join(catalog.ids()) or 'none'}; "
                    "see tools/fetch_urdf.sh)")
            # Fail closed on a mismatched pair: the viewer would draw one arm
            # while the bridge drives another.
            if not model.drivable_by(self.arm_backend):
                raise ConfigError(
                    f"robot_model '{self.robot_model}' ({model.label}) cannot be "
                    f"driven by arm_backend '{self.arm_backend}' (its backends: "
                    f"{', '.join(model.backends) or 'none'}; mock accepts any)")
        self.workspace.validate()
        self.velocity.validate()
        self.xarm.validate()
        self.aubo.validate()
        self.programs.validate()
        self.link_quality.validate()
        self.capture.validate()
        if self.protocol_version not in (3, 4):
            raise ConfigError(f"protocol_version must be 3 or 4 (got {self.protocol_version})")
        # Peak commanded linear step per servo tick must stay under the SDK limit.
        max_step = self.safety_factor * self.velocity.max_linear_mm_s / self.servo_hz
        if not max_step < self.MAX_LINEAR_STEP_MM:
            raise ConfigError(
                f"max linear step {max_step:.2f} mm/tick must be < "
                f"{self.MAX_LINEAR_STEP_MM:.0f} mm (reduce max_linear_mm_s or raise servo_hz)")

    @property
    def servo_dt(self) -> float:
        return 1.0 / self.servo_hz

    @property
    def watchdog_s(self) -> float:
        return self.watchdog_ms / 1000.0

    @property
    def motion_permit_max_age_s(self) -> float:
        return self.motion_permit_max_age_ms / 1000.0


# ---- YAML loading (WAN plan S1) ------------------------------------------
#
# `--config <yaml>` schema mirrors the dataclasses exactly: top-level scalar
# keys named after ArmBridgeConfig fields plus optional `workspace:` and
# `velocity:` mappings named after their fields. Unknown keys fail closed.
#
#     arm_backend: mock                       # mock | xarm  (config-only selector)
#     robot_model: uf850                      # the arm type; empty = driver default
#     servo_hz: 250
#     watchdog_ms: 150
#     stun_server: "stun.example.com:3478"    # bare host:port; empty/absent = LAN
#     xarm_ip: "192.168.1.100"                # bare host; required when arm_backend=xarm
#     dashboard_port: 8081                    # absent = dashboard off
#     dashboard_bind: "127.0.0.1"             # loopback default; LAN exposure is explicit
#     camera_view: false                      # true = advertise the live camera tabs
#     camera_signaling: ""                    # empty = ws://<host>:8443/rtc; else explicit URL
#                                             #   (the tabs use /monitor on its host:port)
#     workspace:
#       x_min: -700.0
#       x_max: 700.0
#       keepouts: [{shape: cylinder, name: base column, cx: 0, cy: 0, radius: 250, z_min: 0, z_max: 350}]
#     velocity:
#       max_linear_mm_s: 250.0
#     xarm:                                    # read only when arm_backend=xarm
#       gripper_scale: 10.0
#       tcp_offset: [0, 0, 0, 0, 0, 0]
#       payload_kg: 0.0
#     aubo:                                    # read only when arm_backend=aubo
#       host: "192.168.1.10"
#       tcp_offset: [0, 0, 0, 0, 0, 0]         # mm + deg
#       payload_kg: 0.0
#       payload_cog: [0, 0, 0]                 # mm
#       program_features: [line, pause]        # verified program features only
#     programs:                                # Programs tab (dashboard)
#       dir: programs                          # relative = to the repo root
#       watchdog_s: 5                          # stop a run nobody watches; 0 = off
#     link_quality:                            # dashboard link readout (display only)
#       rtt_fair_ms: 80
#       rtt_poor_ms: 150
#     protocol_version: 3                      # HELLO reply; 4 = rev 0x04 heartbeats
#     capture:                                 # arm records -> the capture node
#       enabled: false
#       socket_name: teleop-capture-arm      # abstract AF_UNIX name


def _nested_from_mapping(cls, mapping, section: str):
    if not isinstance(mapping, dict):
        raise ConfigError(f"config section '{section}' must be a mapping")
    allowed = {f.name for f in fields(cls)}
    unknown = sorted(set(mapping) - allowed)
    if unknown:
        raise ConfigError(f"unknown {section} key(s): {', '.join(unknown)}")
    return cls(**mapping)


def _commissioned_from_mapping(cls, mapping, section: str):
    """Like `_nested_from_mapping` but records which keys the YAML explicitly set,
    so `require_commissioned()` can refuse silently-defaulted commissioning values."""
    obj = _nested_from_mapping(cls, mapping, section)
    object.__setattr__(obj, "_provided", frozenset(mapping.keys()))
    return obj


def _xarm_from_mapping(mapping) -> XArmTuning:
    return _commissioned_from_mapping(XArmTuning, mapping, "xarm")


def config_from_mapping(mapping: dict) -> ArmBridgeConfig:
    """Build a config from a parsed YAML mapping. NOT validated here; callers
    run validate() after any CLI overrides are applied."""
    if not isinstance(mapping, dict):
        raise ConfigError("config root must be a mapping")
    scalar_names = ({f.name for f in fields(ArmBridgeConfig)}
                    - {"workspace", "velocity", "xarm", "aubo", "programs", "link_quality",
                       "capture"})
    kwargs: dict = {}
    for key, value in mapping.items():
        if key == "workspace":
            kwargs["workspace"] = _nested_from_mapping(WorkspaceLimits, value, "workspace")
        elif key == "velocity":
            kwargs["velocity"] = _nested_from_mapping(VelocityLimits, value, "velocity")
        elif key == "xarm":
            kwargs["xarm"] = _xarm_from_mapping(value)
        elif key == "aubo":
            kwargs["aubo"] = _commissioned_from_mapping(AuboTuning, value, "aubo")
        elif key == "programs":
            kwargs["programs"] = _nested_from_mapping(ProgramsConfig, value, "programs")
        elif key == "link_quality":
            kwargs["link_quality"] = _nested_from_mapping(LinkQualityConfig, value, "link_quality")
        elif key == "capture":
            kwargs["capture"] = _nested_from_mapping(CaptureConfig, value, "capture")
        elif key in scalar_names:
            kwargs[key] = value
        else:
            raise ConfigError(f"unknown config key '{key}'")
    return ArmBridgeConfig(**kwargs)


def load_config(path: str) -> ArmBridgeConfig:
    """Load a YAML config file. PyYAML ships in the dev image (python3-yaml);
    a missing module or file is a ConfigError, never a silent default."""
    try:
        import yaml
    except ImportError as exc:
        raise ConfigError(
            "--config requires PyYAML (apt: python3-yaml / pip: pyyaml)") from exc
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except OSError as exc:
        raise ConfigError(f"cannot read config file '{path}': {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"config file '{path}' is not valid YAML: {exc}") from exc
    return config_from_mapping(data if data is not None else {})


def resolve_config(config_path: str | None = None,
                   stun_server: str | None = None) -> ArmBridgeConfig:
    """Defaults < YAML file < CLI override, then validate.

    `stun_server=None` means "leave the file/default value"; an explicit value
    (including empty string, which forces LAN mode) wins. `arm_backend` and
    `xarm_ip` are config-only (no CLI override).
    """
    config = load_config(config_path) if config_path else ArmBridgeConfig()
    if stun_server is not None:
        config = replace(config, stun_server=stun_server)
    config.validate()
    return config
