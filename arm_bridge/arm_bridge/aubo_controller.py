"""Real AUBO controller (ARCS + pyaubo_sdk) -- `arm_backend: aubo`.

The concrete `ArmController` for an AUBO arm on an ARCS controller (this
station: an i5H on a CB-M; facts and measurements in aubo-control-notes.md).
SDK-gated like the xArm driver: `pyaubo_sdk` is imported lazily by the SDK
factory, so this module imports and the unit tests run without it (tests
inject a fake SDK module).

Threads (the bridge invariant: the servo loop is the sole actuator):
- The COMMAND RPC connection is used only from the servo thread -- commands,
  actions, the per-tick `service()` hook -- plus `connect()` before the loop.
- State comes from a POLLER thread with its own read-only RPC connection
  (aubo.state_hz, default 100 Hz: joints, joint speeds, TCP pose, robot and
  safety mode; plus collision / joint faults / running program / free-drive at
  5 Hz). It writes a cache under `_lock`; `read_snapshot()` reads that cache and
  never calls the SDK, so the status thread can call it at 30 Hz. A cache older
  than `aubo.report_stale_ms` reports UNKNOWN, which the servo loop never
  drives; the poller reconnects on its own.
- Not RTDE: its Python subscriber callback runs on the SDK's thread, and
  tearing a subscription down (removeTopic, even disconnect) deadlocked against
  it on the arm (GIL vs. the SDK's own wait). RPC calls release the GIL
  (measured), so polling has no such failure mode.

Pose units: the bridge speaks mm + degrees, ARCS metres + radians. AUBO's
[rx, ry, rz] is the same R = Rz*Ry*Rx roll/pitch/yaw as the xArm's and its
base frame is the URDF base_link (verified on the live arm, 2026-10-11), so the
conversion is units only and the Quest client drives it unchanged.

Driving (`set_servo_cartesian`, called EVERY driving tick -- streams_every_tick):
the target is rate-limited from the last command (Cartesian clamp from the
config velocity limits, then a per-joint speed cap), solved with the
controller's IK seeded from the last commanded joints, validated (IK result,
branch flip, bridge-owned joint limits) and queued with `trackJoint` (no servo
mode, no auto-exit; holding = not feeding). An invalid target is HELD -- the
arm stays at its last good command and STATUS carries a teleop warn code --
never clamped into a different motion. Nothing upstream limits speed for this
backend (the xArm relied on its own planner), so these limits are the cap.

`connect()` is a fail-closed gate: SDK/controller interface versions match,
the robot's type+subtype is the configured `robot_model`, and the
commissioned TCP offset/payload are on the controller. It never powers the arm
on and never clears a fault -- both are explicit operator actions.
"""

from __future__ import annotations

import logging
import math
import threading
import time

from . import error_codes as ec
from . import safety
from .arm_interface import ArmController, ArmFault, ArmSnapshot, Pose
from .aubo_program import AuboProgramMixin
from .config import ArmBridgeConfig
from .protocol import ArmState
from .robot_models import default_catalog

log = logging.getLogger("arm_bridge.aubo")

DOF = 6
SLOW_POLL_PERIOD_S = 0.2           # collision / joint faults / program / free-drive
POLL_RECONNECT_BACKOFF_S = 1.0     # poller lost its connection -> retry this often
POLL_JOIN_TIMEOUT_S = 3.0
ERROR_LOG_PERIOD_S = 5.0
HELD_LOG_PERIOD_S = 1.0
# A pause in the command stream longer than this ends a burst: the next
# command re-seeds from the arm's ACTUAL joints/pose, not the last command.
BURST_GAP_S = 0.25
MOVING_RAD_S = math.radians(0.5)   # any joint faster -> snapshot.moving
# Commanded speed above which an ending burst / a hold gets a smooth stop.
STOP_LIN_MM_S = 2.0
STOP_ANG_DEG_S = 1.0
# A hold that stopped the arm lasts until the target moves this far from the refused one.
HOLD_RELEASE_MM = 5.0
HOLD_RELEASE_DEG = 3.0
CLEAR_SETTLE_S = 1.0
MODE_SETTLE_S = 1.0                # free-drive on/off confirmation
READBACK_TOL = 1e-4                # m / rad

# ARCS return codes (aubo-control-notes.md §5.2).
AUBO_OK = 0
AUBO_BAD_STATE = 1
AUBO_QUEUE_FULL = 2
AUBO_BUSY = 3
AUBO_REQUEST_IGNORE = -13
# The controller's own self-collision prediction refused the point (seen on
# the i5H driven from a folded posture, 2026-10-12). The arm stays put and the
# controller stays Running/Normal, so this is a HOLD (warn 1005), not a fault.
AUBO_TRAJ_SELF_COLLISION = -22
# After a stopJoint/stopLine the controller keeps the trajectory path in the
# stopped state: trackJoint answers -4 (Timeout, the first call) then -3 ("The
# previous command is executing") -- still 2 s later -- until clearPath().
# Measured on the i5H / ARCS 0.32.1, 2026-10-12 (MIMICRY-TELEOP-PLAN.md B0).
AUBO_RC_PREV_EXECUTING = -3
AUBO_RC_TIMEOUT = -4
PATH_STOPPED_RCS = frozenset({AUBO_RC_PREV_EXECUTING, AUBO_RC_TIMEOUT})
CLEAR_PATH_MAX_TRIES = 5           # consecutive clearPath retries before it is a fault
# "Bad state: Robot is not in required safety mode" -- the pendant's safety
# limits tripped; the poller reports the protective stop (err 1003) itself,
# so the stream point is just "not ready" rather than a second, masking fault.
AUBO_BAD_SAFETY_MODE = 211
# Controller refused a stream point because of its state -> "not ready" to the
# servo loop (it re-arms and retries), not a latched fault.
TRANSIENT_RCS = frozenset({AUBO_BAD_STATE, AUBO_BUSY, AUBO_REQUEST_IGNORE,
                           AUBO_BAD_SAFETY_MODE})
NOT_READY_CODE = 1                 # set_servo_cartesian's "controller not ready"

DRIVABLE_SAFETY = ("Normal", "ReducedMode")
ENABLING_MODES = ("ConfirmSafety", "Booting", "PowerOn", "BrakeReleasing")
STOP_SAFETY = ("RobotEmergencyStop", "SystemEmergencyStop", "ProtectiveStop",
               "SafeguardStop", "Violation", "Fault", "Recovery")


def load_sdk():
    """Import pyaubo_sdk on first use (arm_backend: mock never needs it)."""
    try:
        import pyaubo_sdk
    except ImportError as exc:
        raise ArmFault("AUBO SDK not installed (pip: pyaubo-sdk); arm_backend "
                       "'aubo' requires it", ec.SDK_CALL_FAILED) from exc
    return pyaubo_sdk


def sdk_version() -> tuple[int, int] | None:
    """(major, minor) of the installed pyaubo-sdk -- the binding has no
    __version__ (aubo-control-notes.md §5)."""
    try:
        from importlib.metadata import version
        major, minor = version("pyaubo-sdk").split(".")[:2]
        return int(major), int(minor)
    except Exception:  # noqa: BLE001 -- unknown -> the check is skipped (logged)
        return None


def _name(value) -> str:
    """pybind11 enums expose .name; anything else compares by str()."""
    return str(getattr(value, "name", value))


def _finite(values) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def pose_to_si(pose) -> list[float]:
    """Bridge pose (mm, deg) -> ARCS pose (m, rad)."""
    return [pose[0] / 1000.0, pose[1] / 1000.0, pose[2] / 1000.0,
            math.radians(pose[3]), math.radians(pose[4]), math.radians(pose[5])]


def pose_from_si(pose) -> Pose:
    """ARCS pose (m, rad) -> bridge pose (mm, deg)."""
    return (pose[0] * 1000.0, pose[1] * 1000.0, pose[2] * 1000.0,
            math.degrees(pose[3]), math.degrees(pose[4]), math.degrees(pose[5]))


def _fmt_deg(rad_values) -> str:
    return "[" + ", ".join(f"{math.degrees(v):.1f}" for v in rad_values) + "]"


def _fmt_pose(pose) -> str:
    return "(" + ", ".join(f"{v:.1f}" for v in pose) + ")"


class AuboController(AuboProgramMixin, ArmController):
    """See the module doc; controller programs (Programs tab) are the
    AuboProgramMixin in aubo_program.py."""
    streams_every_tick = True
    supports_set_home = True
    supports_power = True
    captures_state = True       # the 100 Hz poller feeds the capture tap itself

    def __init__(self, config: ArmBridgeConfig, sdk_factory=load_sdk,
                 sdk_version_fn=sdk_version, clock=time.monotonic,
                 start_poller: bool = True) -> None:
        self._cfg = config
        self._t = config.aubo
        self._sdk_factory = sdk_factory
        self._sdk_version_fn = sdk_version_fn
        self._clock = clock
        self._sdk = None
        self._rpc = None
        self._state_if = self._config_if = self._manage = None
        self._motion = self._algo = self._runtime = None
        self._robot_if = None
        # State poller: its own connection + thread (tests drive poll_state()).
        self._start_poller = start_poller
        self._poll_rpc = None
        self._poll_handles: tuple | None = None       # (state, manage, runtime)
        self._poll_thread: threading.Thread | None = None
        self._poll_stop = threading.Event()
        self._next_slow_poll = 0.0
        self._poll_log_at = -1e9
        self._connected = False
        model = default_catalog().resolve(config.robot_model)
        self._model_id = model.id if model else (config.robot_model or None)
        lo = tuple(math.radians(v) for v in self._t.joint_min_deg)
        hi = tuple(math.radians(v) for v in self._t.joint_max_deg)
        self._joint_lo, self._joint_hi = lo, hi
        # ---- state cache: the poller (and servo-thread refreshes) write,
        # status/servo threads read -- always under _lock.
        self._lock = threading.Lock()
        self._state_ts: float | None = None
        # Acquisition time of the cached joints (CLOCK_MONOTONIC ns, the
        # midpoint of the joint read) and the data-capture tap, if any.
        self._state_acq_ns = 0
        self._capture = None
        self._q: tuple[float, ...] | None = None      # rad
        self._qd: tuple[float, ...] | None = None     # rad/s
        self._tcp: tuple[float, ...] | None = None    # m, rad
        self._robot_mode = ""
        self._safety_mode = ""
        self._bad_frames = 0
        self._collision = False
        self._faulted_joint: int | None = None
        self._program_running = False
        self._freedrive = False
        # ---- servo-thread driver state (bools/ints read GIL-atomically by
        # the status thread).
        self._control_on = False
        self._last_q: list[float] | None = None       # last commanded joints, rad
        self._last_pose: Pose | None = None           # its pose, mm/deg
        self._last_vel = safety.ZERO_VELOCITY         # its velocity (mm/s, deg/s)
        self._last_qd: list[float] | None = None      # its joint velocity, rad/s
        self._last_cmd_ts = -1e9
        self._held_warn = 0                           # teleop warn while a target is held
        self._held_log_at = -1e9
        self._held_suppressed = 0
        self._hold_stopped = False                    # this hold episode already stopped the arm
        self._stopping_since: float | None = None     # smooth stop issued, arm not yet steady
        self._hold_latch: Pose | None = None          # target that a hold stopped the arm for
        self._clear_path_pending = False              # our stop holds the controller's trajectory path
        self._clear_path_tries = 0
        self._queue_full = 0
        self._replay_q: list[float] | None = None     # last BACKOUT joint-replay point, rad
        self._replay_ts = -1e9
        self._power_seq: tuple[str, float] | None = None
        # Why the last power-up did not finish, and when (wall ms) -- for the
        # dashboard; cleared by the next ENABLE / DISABLE.
        self._power_error: tuple[str, float] | None = None
        self._service_log_at = -1e9
        self._gripper_mm = float(config.workspace.gripper_max)
        self._init_programs()

    # ---- checked SDK boundary --------------------------------------------
    def _rc_str(self, rc) -> str:
        try:
            return f"{rc} ({self._sdk.returnValue2Str(rc)})"
        except Exception:  # noqa: BLE001 -- decoration only
            return str(rc)

    def _invoke(self, op: str, fn, *args):
        """Any SDK call. A raised exception (AuboException, a dropped
        connection) becomes ArmFault -- CONTROLLER_UNREACHABLE when the RPC
        link is gone, SDK_CALL_FAILED otherwise."""
        try:
            return fn(*args)
        except ArmFault:
            raise
        except Exception as exc:  # noqa: BLE001 -- the SDK boundary
            code = ec.SDK_CALL_FAILED if self._rpc_alive() else ec.CONTROLLER_UNREACHABLE
            raise ArmFault(f"AUBO {op} raised {type(exc).__name__}: {exc}", code) from exc

    def _call(self, op: str, fn, *args) -> int:
        """SDK command returning an ARCS status int: anything but 0 -> ArmFault
        (positive 'warnings' such as BAD_STATE/BUSY are refusals too)."""
        rc = self._invoke(op, fn, *args)
        try:
            code = int(rc)
        except (TypeError, ValueError):
            raise ArmFault(f"AUBO {op} returned {rc!r}", ec.SDK_CALL_FAILED) from None
        if code != AUBO_OK:
            raise ArmFault(f"AUBO {op} failed: rc {self._rc_str(code)}", ec.SDK_CALL_FAILED)
        return code

    def _best(self, op: str, fn, *args) -> None:
        """Best-effort housekeeping (stops on a hold path, teardown): logged,
        never raised."""
        try:
            self._call(op, fn, *args)
        except ArmFault as exc:
            log.debug("aubo: %s (best effort) failed: %s", op, exc.message)

    def _rpc_alive(self) -> bool:
        try:
            return bool(self._rpc is not None and self._rpc.hasConnected())
        except Exception:  # noqa: BLE001
            return False

    # ---- lifecycle --------------------------------------------------------
    def connect(self) -> None:
        self._safe_disconnect()           # a reconnect always starts clean
        try:
            self._sdk = self._sdk_factory()
            self._open_rpc()
            self._check_version()
            self._verify_identity()
            self._apply_commissioning()
            self._open_poller()
            self.poll_state(slow=True)            # first state, synchronously
            self._connected = True
            if self._start_poller:
                self._poll_stop.clear()
                self._poll_thread = threading.Thread(
                    target=self._poll_loop, name="aubo-state", daemon=True)
                self._poll_thread.start()
        except ArmFault:
            self._safe_disconnect()
            raise
        except Exception as exc:  # noqa: BLE001 -- connect must only raise ArmFault
            self._safe_disconnect()
            raise ArmFault(f"AUBO connect failed: {exc!r}", ec.SDK_CALL_FAILED) from exc
        with self._lock:
            mode, safety_mode = self._robot_mode, self._safety_mode
        log.info("aubo: connected to %s (%s) robot=%s safety=%s -- %s",
                 self._t.host, self._model_id, mode, safety_mode,
                 "drivable once enabled" if mode == "Running"
                 else "press ENABLE to power on")

    def _open_rpc(self) -> None:
        sdk, t = self._sdk, self._t
        rpc = sdk.RpcClient()
        self._rpc = rpc
        self._invoke("setRequestTimeout", rpc.setRequestTimeout, int(t.request_timeout_ms))
        rc = self._invoke("connect", rpc.connect, t.host, int(t.rpc_port))
        if rc != AUBO_OK or not self._rpc_alive():
            raise ArmFault(f"AUBO RPC connect to {t.host}:{t.rpc_port} failed: rc "
                           f"{self._rc_str(rc)}", ec.CONTROLLER_UNREACHABLE)
        self._call("login", rpc.login, t.user, t.password)
        names = self._invoke("getRobotNames", rpc.getRobotNames)
        if not names:
            raise ArmFault("AUBO controller reports no robots", ec.CONTROLLER_UNREACHABLE)
        robot = self._invoke("getRobotInterface", rpc.getRobotInterface, names[0])
        self._robot_if = robot
        self._state_if = self._invoke("getRobotState", robot.getRobotState)
        self._config_if = self._invoke("getRobotConfig", robot.getRobotConfig)
        self._manage = self._invoke("getRobotManage", robot.getRobotManage)
        self._motion = self._invoke("getMotionControl", robot.getMotionControl)
        self._algo = self._invoke("getRobotAlgorithm", robot.getRobotAlgorithm)
        self._runtime = self._invoke("getRuntimeMachine", rpc.getRuntimeMachine)

    def _check_version(self) -> None:
        """The SDK and controller interface versions must match (a mismatch can
        fail calls with AUBO_VERSION_INCOMPAT); compared on major.minor."""
        info = self._invoke("getSystemInfo", self._rpc.getSystemInfo)
        code = int(self._invoke("getInterfaceVersionCode", info.getInterfaceVersionCode))
        controller = (code // 1_000_000, (code // 1000) % 1000)
        sdk = self._sdk_version_fn()
        if sdk is None:
            log.warning("aubo: pyaubo-sdk version unknown; skipping the interface "
                        "version check (controller %d.%d)", *controller)
            return
        if tuple(sdk) != controller:
            raise ArmFault(
                f"controller interface version {controller[0]}.{controller[1]} != "
                f"pyaubo-sdk {sdk[0]}.{sdk[1]}; re-pin pyaubo-sdk in docker/Dockerfile "
                "(aubo-control-notes.md §5)", ec.VERSION_MISMATCH)

    def _verify_identity(self) -> None:
        """The robot must be the configured robot_model: AUBO names its models
        <type>_<subtype> (aubo_i5H + 10 -> aubo_i5H_10). Compared through the
        catalog (aliases, case) and fail-closed on an unvendored model."""
        cfg = self._config_if
        rtype = str(self._invoke("getRobotType", cfg.getRobotType))
        subtype = str(self._invoke("getRobotSubType", cfg.getRobotSubType))
        reported = f"{rtype}_{subtype}" if subtype else rtype
        catalog = default_catalog()
        want = catalog.resolve(self._cfg.robot_model)
        got = catalog.resolve(reported)
        if want is None:
            raise ArmFault(f"robot_model '{self._cfg.robot_model}' is not a vendored "
                           "model", ec.IDENTITY_MISMATCH)
        if got is None:
            raise ArmFault(f"connected robot '{reported}' has no vendored model -- "
                           "vendor it (tools/fetch_urdf.sh) before driving it",
                           ec.IDENTITY_MISMATCH)
        if got.id != want.id:
            raise ArmFault(f"connected robot '{reported}' ({got.id}) != robot_model "
                           f"'{self._cfg.robot_model}' ({want.id})", ec.IDENTITY_MISMATCH)
        self._model_id = got.id

    def _apply_commissioning(self) -> None:
        """Put the commissioned TCP offset and payload on the controller (the
        config is the one source). Written only when they differ, and the TCP
        offset is read back."""
        cfg, t = self._config_if, self._t
        tcp = pose_to_si(t.tcp_offset)
        current = list(self._invoke("getTcpOffset", cfg.getTcpOffset))
        if len(current) != 6 or any(abs(a - b) > READBACK_TOL for a, b in zip(current, tcp)):
            self._call("setTcpOffset", cfg.setTcpOffset, tcp)
            back = list(self._invoke("getTcpOffset", cfg.getTcpOffset))
            if len(back) != 6 or any(abs(a - b) > READBACK_TOL for a, b in zip(back, tcp)):
                raise ArmFault(f"AUBO TCP offset readback {back} != commissioned {tcp}",
                               ec.SDK_CALL_FAILED)
            log.info("aubo: TCP offset set to %s (mm, deg)", list(t.tcp_offset))
        mass = float(t.payload_kg)
        cog = [v / 1000.0 for v in t.payload_cog]
        payload = self._invoke("getPayload", cfg.getPayload)
        cur_mass, cur_cog = float(payload[0]), list(payload[1])
        inertia_len = len(payload[3]) if len(payload) > 3 else 6
        if abs(cur_mass - mass) > READBACK_TOL or any(
                abs(a - b) > READBACK_TOL for a, b in zip(cur_cog, cog)):
            self._call("setPayload", cfg.setPayload, mass, cog, [0.0, 0.0, 0.0],
                       [0.0] * inertia_len)
            log.info("aubo: payload set to %.3f kg, cog %s mm", mass, list(t.payload_cog))

    # ---- state poller (own connection, own thread; reads only) ------------
    def _open_poller(self) -> None:
        t = self._t
        rpc = self._sdk.RpcClient()
        self._poll_rpc = rpc
        try:
            rpc.setRequestTimeout(int(t.request_timeout_ms))
            rc = rpc.connect(t.host, int(t.rpc_port))
            if rc != AUBO_OK or not rpc.hasConnected():
                raise ArmFault(f"AUBO state connection to {t.host}:{t.rpc_port} "
                               f"failed: rc {self._rc_str(rc)}", ec.CONTROLLER_UNREACHABLE)
            rc = rpc.login(t.user, t.password)
            if rc != AUBO_OK:
                raise ArmFault(f"AUBO state login failed: rc {self._rc_str(rc)}",
                               ec.CONTROLLER_UNREACHABLE)
            robot = rpc.getRobotInterface(rpc.getRobotNames()[0])
            trace = robot.getTrace() if "output" in self._t.program_features else None
            self._poll_handles = (robot.getRobotState(), robot.getRobotManage(),
                                  rpc.getRuntimeMachine(), trace)
        except ArmFault:
            raise
        except Exception as exc:  # noqa: BLE001 -- the SDK boundary
            raise ArmFault(f"AUBO state connection raised {type(exc).__name__}: {exc}",
                           ec.CONTROLLER_UNREACHABLE) from exc

    def _close_poller_rpc(self) -> None:
        rpc = self._poll_rpc
        self._poll_rpc = None
        self._poll_handles = None
        if rpc is not None:
            for op in ("logout", "disconnect"):
                try:
                    getattr(rpc, op)()
                except Exception:  # noqa: BLE001 -- teardown is best effort
                    pass

    def poll_state(self, slow: bool | None = None) -> None:
        """One state read on the poller connection into the cache. `slow`
        forces (True) / skips (False) the 5 Hz extras; None = when due.
        Raises ArmFault on a failed read (the loop reconnects)."""
        handles = self._poll_handles
        if handles is None:
            raise ArmFault("AUBO state connection is down", ec.CONTROLLER_UNREACHABLE)
        state, manage, runtime, trace = handles
        now = self._clock()
        try:
            t_read = time.monotonic_ns()
            q = tuple(float(v) for v in state.getJointPositions())
            acq_ns = (t_read + time.monotonic_ns()) // 2
            qd = tuple(float(v) for v in state.getJointSpeeds())
            tcp = tuple(float(v) for v in state.getTcpPose())
            robot_mode = _name(state.getRobotModeType())
            safety_mode = _name(state.getSafetyModeType())
            extras = None
            if slow or (slow is None and now >= self._next_slow_poll):
                self._next_slow_poll = now + SLOW_POLL_PERIOD_S
                extras = self._read_extras(state, manage, runtime, trace)
        except Exception as exc:  # noqa: BLE001 -- the SDK boundary
            raise ArmFault(f"AUBO state read raised {type(exc).__name__}: {exc}",
                           ec.CONTROLLER_UNREACHABLE) from exc
        if len(q) != DOF or len(tcp) != 6 or not _finite(q) or not _finite(tcp):
            with self._lock:
                self._bad_frames += 1
            return
        qd_ok = qd if len(qd) == DOF else None
        with self._lock:
            self._q, self._tcp = q, tcp
            self._qd = qd_ok
            self._robot_mode, self._safety_mode = robot_mode, safety_mode
            self._state_ts = now
            self._state_acq_ns = acq_ns
            if extras is not None:
                self._store_extras(extras)
        tap = self._capture
        if tap is not None:
            # Hand-off only (O(1) append); SI units as read: rad, rad/s, m/rad.
            tap.state(acq_ns, q, qd_ok, tcp, robot_mode, safety_mode,
                      gripper_mm=self._gripper_mm)

    def _read_extras(self, state, manage, runtime, trace=None) -> dict:
        joint_states = [_name(j) for j in state.getJointState()]
        program = self._program_extras(runtime, trace)   # runtime state, line, output
        return {"collision": bool(state.isCollisionOccurred()),
                "faulted_joint": next((i for i, j in enumerate(joint_states)
                                       if j == "Fault"), None),
                "program": program["runtime"] != "Stopped",
                "runtime": program["runtime"], "plan_line": program["plan_line"],
                "freedrive": bool(manage.isFreedriveEnabled())}

    def _store_extras(self, extras: dict) -> None:
        """Caller holds _lock."""
        self._collision = extras["collision"]
        self._faulted_joint = extras["faulted_joint"]
        self._program_running = extras["program"]
        self._runtime_state = extras["runtime"]
        self._plan_line = extras["plan_line"]
        self._freedrive = extras["freedrive"]

    def _poll_loop(self) -> None:
        period = 1.0 / self._t.state_hz
        next_t = time.monotonic()
        while not self._poll_stop.is_set():
            try:
                if self._poll_handles is None:
                    self._open_poller()
                    log.info("aubo: state connection re-established")
                self.poll_state()
            except ArmFault as exc:
                now = time.monotonic()
                if now >= self._poll_log_at:
                    self._poll_log_at = now + ERROR_LOG_PERIOD_S
                    log.warning("aubo: state poll failed (%s); reconnecting", exc.message)
                self._close_poller_rpc()
                self._poll_stop.wait(POLL_RECONNECT_BACKOFF_S)
                next_t = time.monotonic()
                continue
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                self._poll_stop.wait(delay)
            else:
                next_t = time.monotonic()         # fell behind: resync, don't spin

    def _stop_poller(self) -> None:
        self._poll_stop.set()
        thread = self._poll_thread
        self._poll_thread = None
        if thread is not None and thread is not threading.current_thread():
            thread.join(POLL_JOIN_TIMEOUT_S)
        self._close_poller_rpc()

    def _safe_disconnect(self) -> None:
        was_driving = self._control_on or self._freedrive
        self._connected = False
        self._control_on = False
        self._power_seq = None
        self._reset_burst()
        self._stopping_since = None
        self._stop_poller()
        if was_driving and self._rpc_alive() and self._motion is not None:
            self._stop_joint()
            if self._freedrive and self._manage is not None:
                self._best("freedrive(False)", self._manage.freedrive, False)
        self._close_script()
        self._trace_cmd = None
        self._robot_if = None
        rpc = self._rpc
        self._rpc = None
        if rpc is not None:
            for op in ("logout", "disconnect"):
                try:
                    getattr(rpc, op)()
                except Exception:  # noqa: BLE001
                    pass
        with self._lock:
            self._state_ts = None
            self._freedrive = False
            self._program_running = False
            self._runtime_state = "Stopped"
            self._plan_line = None

    def disconnect(self) -> None:
        self._safe_disconnect()

    # ---- state ------------------------------------------------------------
    def _refresh_extras(self) -> None:
        """Fresh extras on the COMMAND connection (servo thread), e.g. for a
        CLEAR verdict that must not wait for the next 5 Hz poll."""
        extras = self._invoke("state extras", self._read_extras, self._state_if,
                              self._manage, self._runtime)
        with self._lock:
            self._store_extras(extras)

    def _modes_rpc(self) -> tuple[str, str]:
        """Fresh robot/safety mode straight from the controller (servo thread)."""
        state = self._state_if
        return (_name(self._invoke("getRobotModeType", state.getRobotModeType)),
                _name(self._invoke("getSafetyModeType", state.getSafetyModeType)))

    def _cached(self) -> dict:
        with self._lock:
            return {"ts": self._state_ts, "acq_ns": self._state_acq_ns,
                    "q": self._q, "qd": self._qd,
                    "tcp": self._tcp, "robot": self._robot_mode,
                    "safety": self._safety_mode, "collision": self._collision,
                    "faulted_joint": self._faulted_joint,
                    "program": self._program_running, "freedrive": self._freedrive}

    def _fresh(self, ts: float | None) -> bool:
        return ts is not None and (self._clock() - ts) * 1000.0 <= self._t.report_stale_ms

    def read_snapshot(self) -> ArmSnapshot:
        s = self._cached()
        pose = pose_from_si(s["tcp"]) if s["tcp"] else (0.0,) * 6
        joints = tuple(math.degrees(v) for v in s["q"]) if s["q"] else None
        if not self._connected or not self._fresh(s["ts"]):
            return ArmSnapshot(pose=pose, gripper=self._gripper_mm,
                               arm_state=ArmState.UNKNOWN, moving=False,
                               servo_active=False, protective_stop=False, joints=joints,
                               ts_mono_ns=s["acq_ns"])
        error_code, protective = ec.aubo_fault(s["robot"], s["safety"], s["collision"],
                                               s["faulted_joint"])
        arm_state = self._map_state(s["robot"], s["safety"], error_code, protective,
                                    s["freedrive"])
        warn = self._held_warn or (ec.REDUCED_MODE if s["safety"] == "ReducedMode" else 0)
        moving = bool(s["qd"]) and max(abs(v) for v in s["qd"]) > MOVING_RAD_S
        return ArmSnapshot(pose=pose, gripper=self._gripper_mm, arm_state=arm_state,
                           moving=moving, servo_active=arm_state == ArmState.SERVO,
                           protective_stop=protective, error_code=error_code,
                           warn_code=warn, joints=joints, ts_mono_ns=s["acq_ns"])

    def _map_state(self, robot_mode: str, safety_mode: str, error_code: int,
                   protective: bool, freedrive: bool) -> ArmState:
        if protective:
            return ArmState.PROTECTIVE_STOP
        if error_code:
            return ArmState.ERROR
        if robot_mode == "Running":
            if safety_mode not in DRIVABLE_SAFETY:
                return ArmState.ENABLING          # e.g. Undefined, transient
            if freedrive:
                return ArmState.READY             # hand-guided: not drivable
            return ArmState.SERVO if self._control_on else ArmState.READY
        if self._power_seq is not None or robot_mode in ENABLING_MODES:
            return ArmState.ENABLING
        if robot_mode in ("", "NoController"):
            return ArmState.UNKNOWN
        return ArmState.DISABLED                  # PowerOff, Idle (brakes on), ...

    def set_capture_tap(self, tap) -> None:
        """The poller hands every state read to `tap` (capture_tap.py),
        stamped at acquisition. None stops it."""
        self._capture = tap

    def robot_model(self) -> str | None:
        return self._model_id

    def power_info(self) -> dict | None:
        """Status thread: the poller's cached modes plus the power-up in flight
        (the sequence tuple is swapped whole on the servo thread)."""
        if not self._connected:
            return None
        s = self._cached()
        seq, err = self._power_seq, self._power_error
        mode = s["robot"]
        if mode == "Running":
            phase = "on"
        elif mode == "BrakeReleasing" or (seq is not None and seq[0] == "startup"):
            phase = "releasing"
        elif mode == "PowerOn" or seq is not None:
            phase = "powering"
        else:
            phase = {"PowerOff": "off", "Idle": "idle"}.get(mode, "other")
        failed = err is not None and phase != "on"   # powered from the pendant since
        return {"phase": phase, "robot_mode": mode, "safety_mode": s["safety"],
                "busy": seq is not None, "error": err[0] if failed else None,
                "error_at_ms": err[1] if failed else None}

    def joint_limits_deg(self) -> list[tuple[float, float]] | None:
        return [(float(a), float(b)) for a, b in zip(self._t.joint_min_deg,
                                                     self._t.joint_max_deg)]

    def read_fault_detail(self) -> str:
        parts = []
        try:
            parts.append("joint_states=" + str(
                [_name(s) for s in self._state_if.getJointState()]))
            parts.append("temperatures=" + str(
                [round(float(v), 1) for v in self._state_if.getJointTemperatures()]))
        except Exception as exc:  # noqa: BLE001 -- diagnostics must not raise
            parts.append(f"detail_err={exc!r}")
        return "; ".join(parts)

    # ---- per-tick hook (servo thread) -------------------------------------
    def service(self, now: float) -> None:
        if not self._connected:
            return
        try:
            self._advance_power(now)
        except Exception as exc:  # noqa: BLE001 -- the hook must never raise
            if self._power_seq is not None:
                self._power_failed(getattr(exc, "message", repr(exc)))
            self._power_seq = None
            if now >= self._service_log_at:
                self._service_log_at = now + ERROR_LOG_PERIOD_S
                log.error("aubo: power-up step failed: %s",
                          getattr(exc, "message", repr(exc)))

    def _power_failed(self, why: str) -> None:
        self._power_error = (why, time.time() * 1000.0)

    def _advance_power(self, now: float) -> None:
        """Power-up runs across ticks: poweron -> Idle -> startup (brake
        release) -> Running, each phase bounded by power_timeout_s."""
        seq = self._power_seq
        if seq is None:
            return
        phase, deadline = seq
        s = self._cached()
        mode, safety_mode = s["robot"], s["safety"]
        if safety_mode in STOP_SAFETY:
            self._power_seq = None
            self._power_failed(f"aborted: safety mode {safety_mode}")
            log.error("aubo: power-up aborted: safety mode %s", safety_mode)
            return
        if mode == "Running":
            self._power_seq = None
            log.info("aubo: powered on and brakes released (Running)")
            return
        if phase == "poweron" and mode == "Idle":
            self._call("startup", self._manage.startup)
            self._power_seq = ("startup", now + self._t.power_timeout_s)
            log.info("aubo: powered on (Idle); releasing brakes")
            return
        if now > deadline:
            self._power_seq = None
            step = "brake release" if phase == "startup" else "power on"
            self._power_failed(f"{step} timed out after {self._t.power_timeout_s:.0f} s "
                               f"(controller {mode})")
            log.error("aubo: power-up timed out in phase %s (robot mode %s)", phase, mode)

    # ---- control mode -----------------------------------------------------
    def _require_connected(self) -> None:
        if not self._connected or self._rpc is None:
            raise ArmFault("AUBO controller not connected", ec.CONTROLLER_UNREACHABLE)

    def _require_running(self, what: str) -> None:
        self._require_connected()
        mode, safety_mode = self._modes_rpc()
        if mode != "Running" or safety_mode not in DRIVABLE_SAFETY:
            raise ArmFault(f"cannot {what}: robot {mode}, safety {safety_mode}",
                           ec.NOT_READY)

    def _stop_acc(self) -> float:
        return math.radians(self._t.stop_joint_acc_deg_s2)

    def _reset_burst(self) -> None:
        self._last_q = None
        self._last_pose = None
        self._last_vel = safety.ZERO_VELOCITY
        self._last_qd = None
        self._held_warn = 0
        self._hold_stopped = False
        self._hold_latch = None

    # ---- stops and the controller's trajectory path -------------------------
    def _stop_joint(self, strict: bool = False) -> None:
        """stopJoint at stop_joint_acc. Every stop leaves the controller's
        trajectory path stopped (trackJoint then answers -4/-3) until
        clearPath(), which the next stream point issues once the arm is
        steady (`_clear_path_pending`)."""
        self._clear_path_pending = True
        if strict:
            self._call("stopJoint", self._motion.stopJoint, self._stop_acc())
        else:
            self._best("stopJoint", self._motion.stopJoint, self._stop_acc())

    def _drop_reference(self) -> None:
        """The next command re-seeds from the actual arm, from rest."""
        self._last_q = None
        self._last_pose = None
        self._last_vel = safety.ZERO_VELOCITY
        self._last_qd = None

    def _clear_path(self) -> int:
        """Release the stopped trajectory path before streaming again (servo
        thread, arm steady). Gives the controller one tick before the next
        point -- the first trackJoint straight after clearPath still answered
        -3 on the arm. Returns 0 (nothing queued this tick)."""
        self._clear_path_tries += 1
        if self._clear_path_tries > CLEAR_PATH_MAX_TRIES:
            self._clear_path_tries = 0
            raise ArmFault("AUBO trackJoint stays refused (previous command executing) "
                           "after repeated clearPath", ec.SDK_CALL_FAILED)
        self._best("clearPath", self._motion.clearPath)
        self._clear_path_pending = False
        self._drop_reference()
        return 0

    # ---- smooth stops (MIMICRY-TELEOP-PLAN.md B2) -----------------------------
    def _commanded_moving(self) -> bool:
        """True when simply ending the stream would change velocity by more
        than one tick's acceleration budget -- i.e. a stop is needed. A
        braking curve's last step is within budget by construction, so
        arriving at rest on a limit needs no stopJoint. Judged on the joints
        (their torque is what collision detection watches), else on the
        Cartesian caps."""
        dt = self._cfg.servo_dt
        if self._last_qd is not None:
            budget = math.radians(self._t.max_joint_acc_deg_s2) * dt
            return max(abs(v) for v in self._last_qd) > budget + 1e-9
        lin, ang = safety.speed(self._last_vel)
        factor, vel = self._cfg.safety_factor, self._cfg.velocity
        return (lin > max(STOP_LIN_MM_S, factor * vel.max_linear_acc_mm_s2 * dt)
                or ang > max(STOP_ANG_DEG_S, factor * vel.max_angular_acc_deg_s2 * dt))

    def _smooth_stop(self, why: str) -> bool:
        """One controller-planned stop if the commanded stream is moving.
        stopJoint decelerates the arm from where it ACTUALLY is (flushing the
        queued points, so it stops short of the last command -- everything
        before that command was already validated). The next command re-seeds
        from the actual arm once it is steady. Returns True when it stopped."""
        if not (self._t.smooth_stop and self._control_on and self._commanded_moving()):
            return False
        lin, ang = safety.speed(self._last_vel)
        self._stop_joint()
        self._stopping_since = self._clock()
        self._drop_reference()
        log.info("aubo: smooth stop (%s) from %.0f mm/s, %.0f deg/s", why, lin, ang)
        return True

    def journal_fields(self) -> dict:
        lin, ang = safety.speed(self._last_vel)
        out = {"cmd": self._last_pose, "v": lin, "w": ang, "held": self._held_warn}
        if self._last_q is not None:
            out["qc"] = [math.degrees(v) for v in self._last_q]
        if self._stopping_since is not None:
            out["stopping"] = True
        if self._hold_latch is not None:
            out["latched"] = True
        return out

    def end_stream(self, reason: str) -> None:
        """Servo loop: a teleop burst ended without leaving control."""
        if not self._connected or self._motion is None:
            return
        self._smooth_stop(reason)

    def _settling(self, s: dict) -> bool:
        """True while a smooth stop is still decelerating the arm: no new
        stream point may be queued on top of the controller's stop."""
        if self._stopping_since is None:
            return False
        qd = s["qd"]
        steady = qd is not None and max(abs(v) for v in qd) <= MOVING_RAD_S
        if steady or self._clock() - self._stopping_since > self._t.smooth_stop_timeout_s:
            self._stopping_since = None
            return False
        return True

    def enable_servo(self) -> None:
        """Take teleop control. Nothing to switch on the controller (trackJoint
        needs no mode); refuses while a pendant/controller program runs. A
        robot that is not Running is not an error here: STATUS shows it and
        set_servo_cartesian answers 'not ready' until it is."""
        self._require_connected()
        if self._cached()["program"]:
            raise ArmFault("a controller program is running (pendant); stop it "
                           "before teleop", ec.PROGRAM_RUNNING)
        self._control_on = True
        self._reset_burst()

    def disable_servo(self) -> None:
        """Leave teleop control and flush queued trackJoint points. Best
        effort: it runs on hold/fault paths, and the queue is only ~100 ms."""
        was_on = self._control_on
        self._control_on = False
        self._reset_burst()
        if was_on and self._rpc_alive() and self._motion is not None:
            self._stop_joint()

    def rearm(self) -> None:
        # Start of a drive burst: re-seed from the actual joints.
        self._reset_burst()

    def set_motion_enabled(self, enabled: bool) -> None:
        self._require_connected()
        self._power_error = None
        if not enabled:
            self._power_seq = None
            self._control_on = False
            self._reset_burst()
            self._stop_joint()
            if self._cached()["freedrive"]:
                self._best("freedrive(False)", self._manage.freedrive, False)
            self._call("poweroff", self._manage.poweroff)
            return
        mode, safety_mode = self._modes_rpc()
        if safety_mode in STOP_SAFETY:
            raise ArmFault(f"cannot enable: safety mode {safety_mode} -- release / "
                           "clear it first", ec.NOT_READY)
        now = self._clock()
        if mode == "Running":
            self._power_seq = None
        elif mode == "PowerOff":
            self._call("poweron", self._manage.poweron)
            self._power_seq = ("poweron", now + self._t.power_timeout_s)
        elif mode == "Idle":
            self._call("startup", self._manage.startup)
            self._power_seq = ("startup", now + self._t.power_timeout_s)
        elif mode in ENABLING_MODES:
            self._power_seq = ("poweron", now + self._t.power_timeout_s)
        else:
            raise ArmFault(f"cannot enable from robot mode {mode}", ec.NOT_READY)

    def set_manual_mode(self, enabled: bool) -> None:
        if enabled:
            self._require_running("enter free-drive")
            self._control_on = False
            self._reset_burst()
            self._stop_joint()
            self._call("freedrive(True)", self._manage.freedrive, True)
        else:
            self._require_connected()
            self._call("freedrive(False)", self._manage.freedrive, False)
        self._await_freedrive(enabled)

    def _await_freedrive(self, enabled: bool) -> None:
        deadline = time.monotonic() + MODE_SETTLE_S
        while True:
            state = bool(self._invoke("isFreedriveEnabled", self._manage.isFreedriveEnabled))
            if state == enabled:
                with self._lock:
                    self._freedrive = state
                return
            if time.monotonic() >= deadline:
                raise ArmFault(f"free-drive did not turn {'on' if enabled else 'off'}",
                               ec.TIMEOUT)
            time.sleep(0.02)

    def _move_joint(self, op: str, q, acc: float, vel: float) -> None:
        """Planned moveJoint that first releases a stopped trajectory path
        (see _stop_joint) and retries the controller's brief -3 straight
        after clearPath. Blocking up to ~0.15 s on the servo thread."""
        if self._clear_path_pending:
            self._best("clearPath", self._motion.clearPath)
            self._clear_path_pending = False
        for attempt in range(6):
            rc = self._invoke(op, self._motion.moveJoint, q, acc, vel, 0.0, 0.0)
            try:
                code = int(rc)
            except (TypeError, ValueError):
                raise ArmFault(f"AUBO {op} returned {rc!r}", ec.SDK_CALL_FAILED) from None
            if code == AUBO_OK:
                return
            if code in PATH_STOPPED_RCS and attempt < 5:
                if attempt == 0:
                    self._best("clearPath", self._motion.clearPath)
                time.sleep(0.03)
                continue
            raise ArmFault(f"AUBO {op} failed: rc {self._rc_str(code)}", ec.SDK_CALL_FAILED)

    def go_home(self) -> None:
        """moveJoint to the controller's home position (queued, asynchronous;
        scaled by the pendant speed fraction). The servo loop stands down
        until the arm stops moving."""
        if self._cached()["freedrive"]:
            self.set_manual_mode(False)           # HOME = leave free-drive
        self._require_running("move home")
        home = list(self._invoke("getHomePosition", self._config_if.getHomePosition))
        self._check_joint_target(home, "home position")
        self._control_on = False
        self._reset_burst()
        self._move_joint("moveJoint(home)", home, math.radians(self._t.home_acc_deg_s2),
                         math.radians(self._t.home_speed_deg_s))
        log.info("aubo: moving home to %s deg", _fmt_deg(home))

    def stop_motion(self) -> None:
        self._require_connected()
        self._stop_joint(strict=True)

    supports_retreat = True

    def retreat_to(self, joints_deg, pose) -> None:
        """RETREAT: a planned joint move (home speed) to the servo loop's
        nearest-in-envelope target -- joints directly, or a TCP pose solved
        with the controller's IK from the current joints (a branch flip is
        refused like in teleop). Leaves free-drive first, like HOME."""
        if self._cached()["freedrive"]:
            self.set_manual_mode(False)
        self._require_running("retreat")
        if joints_deg is not None:
            q = [math.radians(float(v)) for v in joints_deg]
        elif pose is not None:
            seed = self._cached()["q"]
            if seed is None:
                raise ArmFault("no joint state to seed the retreat IK", ec.NOT_READY)
            sol, rc = self._invoke("inverseKinematics", self._algo.inverseKinematics,
                                   list(seed), pose_to_si(pose))
            q = list(sol) if sol is not None else []
            if int(rc) != AUBO_OK or len(q) != DOF or not _finite(q):
                raise ArmFault(f"no IK solution for the retreat pose {_fmt_pose(pose)} "
                               f"(rc {self._rc_str(rc)})", ec.KINEMATIC)
            biggest = max(abs(a - b) for a, b in zip(q, seed))
            if biggest > math.radians(self._t.ik_flip_deg):
                raise ArmFault(f"retreat pose needs a {math.degrees(biggest):.0f} deg joint "
                               "jump (configuration flip) -- hand-guide the arm out instead",
                               ec.KINEMATIC)
        else:
            raise ArmFault("retreat needs a joint or pose target", ec.NOT_READY)
        self._check_joint_target(q, "retreat target")
        self._control_on = False
        self._reset_burst()
        self._move_joint("moveJoint(retreat)", q, math.radians(self._t.home_acc_deg_s2),
                         math.radians(self._t.home_speed_deg_s))
        log.info("aubo: retreating to %s deg", _fmt_deg(q))

    def set_home_here(self) -> tuple[bool, int, str]:
        try:
            self._require_connected()
            q = list(self._invoke("getJointPositions", self._state_if.getJointPositions))
            self._check_joint_target(q, "current position")
            self._call("setHomePosition", self._config_if.setHomePosition, q)
            back = list(self._invoke("getHomePosition", self._config_if.getHomePosition))
            if len(back) != DOF or any(abs(a - b) > READBACK_TOL for a, b in zip(back, q)):
                return (False, ec.SDK_CALL_FAILED,
                        f"home readback {_fmt_deg(back)} != {_fmt_deg(q)}")
            log.info("aubo: home position set to %s deg", _fmt_deg(q))
            return (True, 0, f"home set to {_fmt_deg(q)} deg")
        except ArmFault as exc:
            return (False, exc.code, exc.message)

    def _check_joint_target(self, q, what: str) -> None:
        if len(q) != DOF or not _finite(q):
            raise ArmFault(f"{what} {q!r} is not a finite 6-vector", ec.SDK_CALL_FAILED)
        bad = self._outside_limits(q)
        if bad is not None:
            raise ArmFault(f"{what} {_fmt_deg(q)} deg: J{bad + 1} outside the "
                           "configured aubo joint limits", ec.JOINT_LIMIT)

    def _shape_joint_step(self, ref_q, q, dt: float) -> tuple[list[float] | None, int | None]:
        """(the joint step after the per-joint acceleration cap and limit
        braking -- None when `q` already satisfies both --, the index of a
        joint resting ON its limit that the step wanted to push past, else
        None). A burst starts from rest (no previous joint velocity)."""
        a = math.radians(self._t.max_joint_acc_deg_s2)
        prev = self._last_qd if self._last_qd is not None else [0.0] * DOF
        out, changed, at_limit = [], False, None
        for i, (r, t) in enumerate(zip(ref_q, q)):
            v = (t - r) / dt
            lo, hi = prev[i] - a * dt, prev[i] + a * dt
            v_new = min(max(v, lo), hi)
            # Room to this joint's limit in the direction of travel; the
            # discrete braking curve lands exactly ON the limit.
            room = (self._joint_hi[i] - r) if v_new > 0.0 else (r - self._joint_lo[i])
            v_lim = min(safety.brake_speed(max(room, 0.0), a, dt), max(room, 0.0) / dt)
            if abs(v_new) > v_lim:
                if room <= 1e-9 and at_limit is None:
                    at_limit = i
                v_new = math.copysign(v_lim, v_new)
            if abs(v_new - v) > 1e-12:
                changed = True
            # Float guard: never a hair past the limit from rounding.
            out.append(min(max(r + v_new * dt, self._joint_lo[i]), self._joint_hi[i])
                       if abs(v_new) > 0.0 else r + v_new * dt)
        return (out if changed else None), at_limit

    def _elbow_scale(self, r: float, t: float, dt: float) -> tuple[float, str]:
        """(the fraction of this tick's joint step to take, a hold reason) for
        the elbow guard, given J3 now (`r`) and after the full step (`t`).
        A step that straightens the elbow (or crosses J3 = 0) follows the
        discrete braking curve at max_joint_acc, landing exactly ON the guard;
        resting there, it holds. Bending steps pass, so a pull-back leaves the
        guard -- except deep in the band (|J3| under half the guard: a
        restart, free-drive or program left the elbow nearly straight), where
        the nearest IK branch may be the mirrored elbow: HOME instead."""
        guard_deg = self._t.elbow_min_deg
        guard = math.radians(guard_deg)
        if guard <= 0.0:
            return 1.0, ""
        if abs(r) < guard / 2.0:
            return 0.0, (f"J3 {math.degrees(r):.1f} deg: the elbow is nearly straight "
                         f"(guard {guard_deg:g} deg) -- press HOME")
        if abs(t) >= abs(r) and t * r >= 0.0:
            return 1.0, ""                        # bending (or J3 unchanged)
        room = abs(r) - guard
        if room <= 1e-9:
            return 0.0, f"J3 at the {guard_deg:g} deg elbow guard: the arm is fully stretched"
        a = math.radians(self._t.max_joint_acc_deg_s2)
        allowed = min(safety.brake_speed(room, a, dt) * dt, room)
        travel = abs(t - r)
        return (1.0 if travel <= allowed else allowed / travel), ""

    def _outside_limits(self, q) -> int | None:
        return next((i for i, v in enumerate(q)
                     if not self._joint_lo[i] <= v <= self._joint_hi[i]), None)

    def clear_error(self) -> tuple[bool, int, str]:
        """Operator CLEAR/RECOVER: reconnect if the controller link was lost,
        unlock a protective stop, then report truthfully. Never powers on --
        a cleared-but-powered-off arm is ok=True with a 'press ENABLE' note
        (STATUS shows DISABLED)."""
        try:
            if not self._rpc_alive():
                self.connect()                    # re-verifies identity too
            mode, safety_mode = self._modes_rpc()
            if safety_mode == "ProtectiveStop":
                self._call("setUnlockProtectiveStop", self._manage.setUnlockProtectiveStop)
                deadline = time.monotonic() + CLEAR_SETTLE_S
                while safety_mode == "ProtectiveStop" and time.monotonic() < deadline:
                    time.sleep(0.05)
                    mode, safety_mode = self._modes_rpc()
            self._reset_burst()
            self._refresh_extras()
            s = self._cached()
            if s["program"]:
                # Belt and braces under the servo loop's abort-first rule: a
                # fresh read says a program is still loaded -- unlocking now
                # could let it continue.
                return (False, ec.PROGRAM_RUNNING,
                        "a controller program is still running -- stop it first")
            error_code, _ = ec.aubo_fault(mode, safety_mode, s["collision"],
                                          s["faulted_joint"])
            if error_code:
                return (False, error_code, f"fault persists: {ec.error_label(error_code)} "
                                           f"(robot {mode}, safety {safety_mode})")
            if mode != "Running":
                return (True, 0, f"cleared; robot is {mode} -- press ENABLE to power on")
            return (True, 0, "")
        except ArmFault as exc:
            return (False, exc.code, exc.message)

    # ---- drive ------------------------------------------------------------
    def _ready_to_drive(self, s: dict) -> bool:
        return (self._connected and self._control_on and not s["freedrive"]
                and self._fresh(s["ts"]) and s["q"] is not None and s["tcp"] is not None
                and s["robot"] == "Running" and s["safety"] in DRIVABLE_SAFETY)

    def set_servo_cartesian(self, pose: Pose) -> int:
        now = self._clock()
        s = self._cached()
        if not self._ready_to_drive(s):
            return NOT_READY_CODE
        if self._settling(s):
            return 0                              # holding while a smooth stop completes
        if self._hold_latch is not None:
            # A hold stopped the arm: stay stopped while the operator's target
            # is still the one that was refused, instead of re-seeding from the
            # (short-stopped) arm and creeping back into the same limit.
            lin, ang = safety.pose_distance(pose, self._hold_latch)
            if lin < HOLD_RELEASE_MM and ang < HOLD_RELEASE_DEG:
                return 0
            self._hold_latch = None
        if self._clear_path_pending:
            return self._clear_path()
        dt = self._cfg.servo_dt
        if self._last_q is None or now - self._last_cmd_ts > BURST_GAP_S:
            ref_q = list(s["q"])                  # burst start: from the actual arm
            ref_pose = pose_from_si(s["tcp"])
            ref_vel = safety.ZERO_VELOCITY        # (a > 0.25 s pause: the arm has stopped)
        else:
            ref_q, ref_pose, ref_vel = self._last_q, self._last_pose, self._last_vel
        # 1. Cartesian rate + acceleration limit (config velocity limits x
        #    safety_factor): a jumped or reversing target becomes a smooth
        #    ramp -- nothing upstream limits it, and instant velocity steps
        #    tripped the controller's collision detection (2026-10-12).
        #    The commanded path and its braking path stay inside the position
        #    envelope: near a boundary the ramp brakes (soft wall), never
        #    reverses instantly (safety.envelope_step).
        cmd_pose, _vel = safety.envelope_step(ref_pose, ref_vel, pose, dt, self._cfg)
        if self._last_pose is not None and ref_pose is self._last_pose and max(
                safety.pose_distance(cmd_pose, self._last_pose)) < 1e-3:
            self._last_cmd_ts = now               # at the target: nothing new to queue
            self._last_vel = safety.ZERO_VELOCITY
            return 0
        # 2. Controller IK, seeded with the reference joints for continuity.
        sol, rc = self._invoke("inverseKinematics", self._algo.inverseKinematics,
                               list(ref_q), pose_to_si(cmd_pose))
        q = list(sol) if sol is not None else []
        if int(rc) != AUBO_OK or len(q) != DOF or not _finite(q):
            return self._hold(now, ec.TARGET_NO_IK,
                              f"no IK solution (rc {self._rc_str(rc)}) for {_fmt_pose(cmd_pose)}", target=pose)
        dq = [a - b for a, b in zip(q, ref_q)]
        biggest = max(abs(d) for d in dq)
        # 3. Branch flip (singularity crossing): never follow it.
        if biggest > math.radians(self._t.ik_flip_deg):
            return self._hold(now, ec.TARGET_IK_FLIP,
                              f"IK jumps {math.degrees(biggest):.0f} deg in one tick "
                              f"at {_fmt_pose(cmd_pose)}", target=pose)
        # 3b. Elbow guard: |J3| never closes below elbow_min_deg (a straight
        #     elbow is singular). The WHOLE step slows so the arm brakes onto
        #     the guard on the commanded path -- braking J3 alone (4b) would
        #     let the other joints carry the tool off it, up to ~100 mm down
        #     at full stretch on the i5H.
        scale, why = self._elbow_scale(ref_q[2], q[2], dt)
        if why:
            return self._hold(now, ec.TARGET_JOINT_LIMIT, why, target=pose)
        # 4. Per-joint speed cap: near a singularity the arm lags, never whips.
        max_step = math.radians(self._t.max_joint_speed_deg_s) * dt
        if biggest > max_step:
            scale = min(scale, max_step / biggest)
        if scale < 1.0:
            q = [r + d * scale for r, d in zip(ref_q, dq)]
            fk, frc = self._invoke("forwardKinematics", self._algo.forwardKinematics, q)
            if int(frc) == AUBO_OK and fk is not None and len(fk) == 6 and _finite(fk):
                cmd_pose = pose_from_si(fk)
            else:
                cmd_pose = tuple(r + (c - r) * scale for r, c in zip(ref_pose, cmd_pose))
        # 4b. Per-joint acceleration cap and limit braking (B4): each joint's
        #     velocity changes by at most max_joint_acc * dt per tick, and a
        #     joint heading for its bridge limit slows so it can stop on it
        #     (|qd| <= sqrt(2 a d)) -- near a singularity a gentle Cartesian
        #     move can still demand a violent joint move, and joint torque is
        #     what the controller's collision detection watches.
        shaped, at_limit = self._shape_joint_step(ref_q, q, dt)
        if at_limit is not None and max(abs(a - b) for a, b in zip(shaped or q, ref_q)) < 1e-9:
            # Braked onto a joint limit and the target still lies beyond it:
            # rest there with the joint-limit warning (no stop needed, the
            # braking already brought the stream to rest).
            return self._hold(now, ec.TARGET_JOINT_LIMIT,
                              f"J{at_limit + 1} at its limit "
                              f"[{self._t.joint_min_deg[at_limit]}, "
                              f"{self._t.joint_max_deg[at_limit]}]", target=pose)
        if shaped is not None:
            q = shaped
            fk, frc = self._invoke("forwardKinematics", self._algo.forwardKinematics, q)
            if int(frc) == AUBO_OK and fk is not None and len(fk) == 6 and _finite(fk):
                cmd_pose = pose_from_si(fk)
        # 5. Bridge-owned joint limits (the controller allows +/-360 everywhere).
        bad = self._outside_limits(q)
        if bad is not None:
            return self._hold(now, ec.TARGET_JOINT_LIMIT,
                              f"J{bad + 1} would reach {math.degrees(q[bad]):.1f} deg, "
                              f"outside [{self._t.joint_min_deg[bad]}, "
                              f"{self._t.joint_max_deg[bad]}]", target=pose)
        rc = int(self._invoke("trackJoint", self._motion.trackJoint, q, dt,
                              float(self._t.track_smooth_scale),
                              float(self._t.track_delay_scale)))
        if rc in PATH_STOPPED_RCS:
            # The path is still held by a stop issued elsewhere (a program, the
            # pendant, an earlier session): clear it next tick and retry --
            # not a fault (it was, 2026-10-12: every session after the first).
            self._clear_path_pending = True
            self._drop_reference()
            log.info("aubo: trackJoint rc %s -- clearing the stopped trajectory path",
                     self._rc_str(rc))
            return 0
        if rc == AUBO_OK:
            self._clear_path_tries = 0
            if self._held_warn:
                log.info("aubo: target valid again; moving (%s)", ec.warn_label(self._held_warn))
            self._last_q, self._last_pose, self._last_cmd_ts = q, cmd_pose, now
            # What was really commanded (a joint-speed cap may have shortened
            # the step) is the velocity the next tick ramps from.
            self._last_vel = safety.step_velocity(ref_pose, cmd_pose, dt)
            self._last_qd = [(a - b) / dt for a, b in zip(q, ref_q)]
            self._held_warn = 0
            self._hold_stopped = False
            return 0
        if rc == AUBO_QUEUE_FULL:
            # Point dropped (streaming too fast); the next tick sends a fresh one.
            self._queue_full += 1
            if self._queue_full in (1, 10, 100) or self._queue_full % 1000 == 0:
                log.warning("aubo: trackJoint queue full (%d points dropped)", self._queue_full)
            return 0
        if rc == AUBO_TRAJ_SELF_COLLISION:
            # Held like an IK refusal: the last good command stands, the Quest
            # sees warn 1005, and a target away from the body moves again.
            return self._hold(now, ec.TARGET_SELF_COLLISION,
                              f"controller predicts a self collision at {_fmt_pose(cmd_pose)} "
                              f"(joints {_fmt_deg(q)})", target=pose)
        if rc in TRANSIENT_RCS:
            return NOT_READY_CODE
        raise ArmFault(f"AUBO trackJoint failed: rc {self._rc_str(rc)}", ec.SDK_CALL_FAILED)

    def _hold(self, now: float, warn: int, detail: str, target: Pose | None = None) -> int:
        """Refuse this target: the arm stays at its last good command; STATUS
        carries `warn` until a valid target moves it again. Returns 0 (the
        servo loop sees a sent command, not a not-ready controller).
        `_last_q`/`_last_pose` are kept so the next tick recomputes from the
        SAME reference (no creep toward an invalid target), and `_last_cmd_ts`
        is deliberately not refreshed: after BURST_GAP_S of holds the burst
        re-seeds from the actual joints, which equal the last good command
        because the arm has been standing there. The ramp velocity is zeroed:
        the arm stops at the last good command, so the next valid target
        accelerates from rest."""
        self._held_warn = warn
        if not self._hold_stopped:
            # First refusal of this episode while the stream is moving: stop
            # smoothly (the reference then re-seeds from the steady arm)
            # instead of freezing the stream at full speed.
            self._hold_stopped = True
            if self._smooth_stop(f"held: {ec.warn_label(warn)}") and target is not None:
                self._hold_latch = target
        self._last_vel = safety.ZERO_VELOCITY
        self._last_qd = None
        if now >= self._held_log_at:
            suffix = f" ({self._held_suppressed} more suppressed)" if self._held_suppressed else ""
            log.warning("aubo: target held -- %s: %s%s", ec.warn_label(warn), detail, suffix)
            self._held_log_at = now + HELD_LOG_PERIOD_S
            self._held_suppressed = 0
        else:
            self._held_suppressed += 1
        return 0

    supports_joint_replay = True

    def joint_replay_limits_deg(self) -> tuple[float, float]:
        # The replay is a slow undo, like HOME and RETREAT (streamed, so not
        # scaled by the pendant speed fraction).
        return (float(self._t.home_speed_deg_s), float(self._t.home_acc_deg_s2))

    def set_servo_joints(self, joints_deg) -> int:
        """BACKOUT joint replay: one point of the servo loop's time-
        parameterized path, straight to trackJoint -- no IK, so it works at
        and through a singularity (every point is one the arm stood at). Only
        the bridge joint limits and a step-size sanity check apply. Returns 0
        sent, NOT_READY_CODE to retry the same point, else a refusal code."""
        now = self._clock()
        s = self._cached()
        if not self._ready_to_drive(s) or self._settling(s):
            return NOT_READY_CODE
        if self._clear_path_pending:
            self._clear_path()
            return NOT_READY_CODE
        q = [math.radians(float(v)) for v in joints_deg]
        if len(q) != DOF or not _finite(q):
            return ec.SDK_CALL_FAILED
        bad = self._outside_limits(q)
        if bad is not None:
            log.warning("aubo: back-out point refused: J%d %.1f deg outside the bridge limits",
                        bad + 1, math.degrees(q[bad]))
            return ec.TARGET_JOINT_LIMIT
        dt = self._cfg.servo_dt
        fresh = self._replay_q is not None and now - self._replay_ts <= BURST_GAP_S
        ref = self._replay_q if fresh else list(s["q"])
        step = max(abs(a - b) for a, b in zip(q, ref))
        # The path is profiled well under the teleop joint speed cap; the slack
        # covers the first point against the ACTUAL arm (tracking error).
        if step > math.radians(self._t.max_joint_speed_deg_s) * dt + math.radians(1.0):
            log.warning("aubo: back-out point refused: a %.1f deg joint step in one tick "
                        "(the arm is not where the replay expects)", math.degrees(step))
            return ec.TARGET_IK_FLIP
        rc = int(self._invoke("trackJoint", self._motion.trackJoint, q, dt,
                              float(self._t.track_smooth_scale),
                              float(self._t.track_delay_scale)))
        if rc in PATH_STOPPED_RCS:
            self._clear_path_pending = True
            self._replay_q = None
            return NOT_READY_CODE
        if rc in TRANSIENT_RCS or rc == AUBO_QUEUE_FULL:
            return NOT_READY_CODE
        if rc == AUBO_TRAJ_SELF_COLLISION:
            log.warning("aubo: back-out point refused: controller predicts a self "
                        "collision at joints %s", _fmt_deg(q))
            return ec.TARGET_SELF_COLLISION
        if rc != AUBO_OK:
            raise ArmFault(f"AUBO trackJoint failed: rc {self._rc_str(rc)}", ec.SDK_CALL_FAILED)
        self._last_qd = [(a - b) / dt for a, b in zip(q, ref)]
        self._replay_q, self._replay_ts = q, now
        # Teleop afterwards starts a fresh burst from the actual arm, with no
        # hold left over from before the undo.
        self._last_q = self._last_pose = None
        self._last_vel = safety.ZERO_VELOCITY
        self._last_cmd_ts = now
        self._held_warn = 0
        self._hold_stopped = False
        self._hold_latch = None
        return 0

    def set_gripper(self, gripper_mm: float) -> None:
        # No gripper on this arm: keep the commanded width so STATUS (which
        # reports last-commanded, never measured) stays consistent.
        ws = self._cfg.workspace
        self._gripper_mm = min(max(float(gripper_mm), ws.gripper_min), ws.gripper_max)
