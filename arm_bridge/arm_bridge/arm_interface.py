"""Arm controller abstraction.

The servo loop talks to the arm only through `ArmController`, so the real xArm
SDK (Increment 3, SDK-gated like the streamer's optional RealSense) and the
`MockArmController` used for tests and headless sim are interchangeable. Poses
are xArm base frame, mm and degrees, matching the wire protocol; the real
controller converts at the SDK boundary if needed (rev 3 section 2).
"""

from __future__ import annotations

import abc
import time
from dataclasses import dataclass

from .program_runtime import IDLE, PROGRAM_FEATURES, FakeLuaRuntime, ProgramEvent, ProgramPoll
from .protocol import ArmState

Pose = tuple[float, float, float, float, float, float]


class ArmFault(Exception):
    """A hardware/SDK operation failed (nonzero SDK return, disconnect, timeout,
    outstanding controller error). The servo loop catches this, latches ERROR, and
    holds -- it must never escape and kill the servo thread (rev 3 section 10)."""

    def __init__(self, message: str, code: int = 0) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class ArmSnapshot:
    """Actual arm state read back for STATUS (rev 3 section 5)."""
    pose: Pose
    gripper: float
    arm_state: ArmState
    moving: bool
    servo_active: bool
    protective_stop: bool
    error_code: int = 0
    warn_code: int = 0
    # Joint angles (degrees, joint1..joint6) for the dashboard 3D viewer
    # (SIM-DASHBOARD-PLAN §3). None when the backend has no joint stream
    # (mock; xarm until Phase 4). NOT on the Quest STATUS wire -- dashboard
    # only.
    joints: tuple[float, ...] | None = None
    # CLOCK_MONOTONIC ns at which the state was acquired (data capture,
    # CAPTURE-SYNC-PLAN.md §4). 0 = unknown: the caller stamps its read time.
    ts_mono_ns: int = 0


class ArmController(abc.ABC):
    # Backend capabilities, read by the servo loop / dashboard. Defaults keep
    # the mock/sim/xarm behavior; a wrapper (LoggingArmController) must
    # delegate them explicitly, since these defaults shadow __getattr__.
    #
    # streams_every_tick: call set_servo_cartesian on EVERY driving tick, even
    # when the target is unchanged. For backends that rate-limit toward the
    # target themselves (AUBO): with the default send-on-change gate a driver
    # that moves part-way per call would stall short of a stationary target.
    streams_every_tick: bool = False
    # supports_set_home: set_home_here() is implemented (dashboard button).
    supports_set_home: bool = False
    # supports_retreat: retreat_to() is implemented -- the operator-triggered
    # planned move back inside the envelope (dashboard RETREAT button).
    supports_retreat: bool = False
    # supports_joint_replay: set_servo_joints() + joint_replay_limits_deg()
    # are implemented -- BACKOUT replays the recorded JOINTS (no IK, so it
    # works at a singularity) instead of streaming poses.
    supports_joint_replay: bool = False
    # supports_programs: the program_* methods below are implemented -- the
    # controller runs Lua programs (AUBO; the mock's fake runtime). They are
    # called only by ProgramSupervisor on the servo thread.
    supports_programs: bool = False
    # supports_power: set_motion_enabled() is a real power cycle (motors +
    # brakes) and power_info() reports it -- the dashboard's Arm power menu.
    supports_power: bool = False
    # captures_state: the backend hands its own acquisition-stamped state to
    # the data-capture tap (set_capture_tap); otherwise the status thread
    # records each snapshot it reads.
    captures_state: bool = False

    @abc.abstractmethod
    def connect(self) -> None: ...

    @abc.abstractmethod
    def disconnect(self) -> None: ...

    @abc.abstractmethod
    def enable_servo(self) -> None:
        """Enter servo/cartesian mode (rev 3 section 5 SET_ENABLED(1))."""

    @abc.abstractmethod
    def disable_servo(self) -> None:
        """Leave servo mode and hold (SET_ENABLED(0) / disconnect / watchdog)."""

    @abc.abstractmethod
    def rearm(self) -> None:
        """Re-assert control mode + state 0 after the controller idled to a
        re-armable state (mode-7 MODE_CHANGED / paused). Not error recovery."""

    @abc.abstractmethod
    def clear_error(self) -> tuple[bool, int, str]:
        """Attempt to clear controller error/warning. Returns (ok, code, msg)."""

    def read_fault_detail(self) -> str:
        """Best-effort per-joint fault detail for the journal (e.g. the servo
        debug sub-code that says WHY a servo-motor error tripped: overcurrent vs
        encoder vs comms vs over-temperature). Read-only; called on the servo
        thread from the fault path, so it must not raise. Default: nothing extra
        (mock/sim have no sub-codes). Overridden by the real controller."""
        return ""

    def robot_model(self) -> str | None:
        """Id of the robot model this controller drives, for the dashboard 3D
        viewer: a vendored model under dashboard/static/assets/<id>/ (or an
        alias of one -- see robot_models.py). May change after connect() when
        the backend learns the identity from the robot (e.g. AUBO
        getRobotType()+getRobotSubType() -> 'aubo_i5H_10'). Called on the status
        thread every publish, so it must be a cheap, non-blocking read of a
        cached value and must not raise. None = unknown (viewer shows no arm)."""
        return None

    def joint_limits_deg(self) -> list[tuple[float, float]] | None:
        """Joint limits the BRIDGE enforces for this backend (deg, J1..), for
        display; None = the robot model's URDF limits apply."""
        return None

    def power_info(self) -> dict | None:
        """Power detail for the dashboard (backends with `supports_power`):
        {phase: off|idle|powering|releasing|on|other, robot_mode, safety_mode,
        busy: a power-up is in progress, error: why the last power-up did not
        finish (null once the arm runs), error_at_ms}. Called on the status
        thread every publish: a cheap read of cached values, must not raise.
        None = not reported."""
        return None

    def journal_fields(self) -> dict:
        """Backend detail for the motion journal row of this tick (commanded
        joints, ramp velocity, hold state...). Called on the servo thread;
        must be cheap and must not raise or touch the SDK. Default: none."""
        return {}

    def set_capture_tap(self, tap) -> None:
        """Give a `captures_state` backend the ArmCaptureTap its state
        stream feeds (capture_tap.py). Default: no state stream of its own."""
        return None

    def end_stream(self, reason: str) -> None:
        """Called once by the servo loop when a drive burst ENDS without leaving
        control (deadman released, watchdog / permit / latched hold, envelope
        gate). A backend that streams its own rate-limited trajectory
        (streams_every_tick) must bring the arm to a smooth stop here instead
        of leaving the last queued point as an instant velocity step; it must
        not raise. Default: nothing (the xArm's mode-7 planner decelerates by
        itself when targets stop)."""

    def service(self, now: float) -> None:
        """Per-tick hook, called by the servo loop at the start of EVERY tick on
        the servo thread (so the single-actuator rule holds). Lets a backend
        advance multi-second sequences (e.g. AUBO power-on -> brake release)
        without blocking a tick, and poll slow-changing state. Must be cheap and
        must not raise. Default: nothing."""

    def set_home_here(self) -> tuple[bool, int, str]:
        """Make the arm's CURRENT joint position its home (the pose go_home()
        moves to). Returns (ok, teleop error code, message) like clear_error.
        Only when `supports_set_home`; the default refuses."""
        from .error_codes import UNSUPPORTED
        return (False, UNSUPPORTED, "set home is not supported by this arm backend")

    def retreat_to(self, joints_deg: list[float] | None, pose: Pose | None) -> None:
        """Operator-triggered RETREAT (dashboard, PIN-gated): a slow PLANNED
        move to `joints_deg` (deg, J1..) when given, else to the TCP `pose`
        (mm/deg; the backend solves IK from the current joints). Asynchronous
        like go_home() -- the servo loop stands down until the arm is idle and
        STOP cancels it via stop_motion(). The servo loop computes the target
        (safety.retreat_joints / retreat_pose); the backend only validates and
        executes. Raises ArmFault when it cannot start. Only when
        `supports_retreat`; the default refuses."""
        from .error_codes import UNSUPPORTED
        raise ArmFault("retreat is not supported by this arm backend", UNSUPPORTED)

    def joint_replay_limits_deg(self) -> tuple[float, float]:
        """(max joint speed deg/s, max joint acceleration deg/s^2) for the
        BACKOUT joint replay. Only when `supports_joint_replay`."""
        return (0.0, 0.0)

    def set_servo_joints(self, joints_deg) -> int:
        """Stream one point (deg, J1..) of BACKOUT's time-parameterized joint
        path (safety.joint_path_profile), on the servo thread in control.
        0 = sent; 1 = not sent yet, retry this same point next tick (e.g. a
        stopped trajectory being cleared); any other code = refused (the
        servo loop aborts the replay and holds). Only when
        `supports_joint_replay`; the default refuses."""
        from .error_codes import UNSUPPORTED
        return UNSUPPORTED

    # ---- programs (servo thread only; see program_runtime.py) -------------
    def program_features(self) -> frozenset[str]:
        """Optional features VERIFIED on this controller (subset of
        PROGRAM_FEATURES). Run and Stop need none of them."""
        return frozenset()

    def program_start(self, source: str, *, simulate: bool, speed_fraction: float,
                      breakpoints: tuple[int, ...] = (),
                      line_plan: tuple[int, ...] = ()) -> None:
        """Send `source` to the controller and start it at `speed_fraction`
        (restored by program_finish). Raises ArmFault when it cannot start."""
        from .error_codes import UNSUPPORTED
        raise ArmFault("programs are not supported by this arm backend", UNSUPPORTED)

    def program_control(self, op: str) -> None:
        """'pause' | 'resume' | 'step'. Raises ArmFault."""
        from .error_codes import UNSUPPORTED
        raise ArmFault("programs are not supported by this arm backend", UNSUPPORTED)

    def program_stop(self) -> None:
        """Abort the running program (ours or the pendant's) AND stop motion."""

    def program_set_breakpoints(self, lines: tuple[int, ...]) -> None:
        """Replace the breakpoints of the running program."""

    def program_poll(self) -> ProgramPoll:
        """The controller's program state. Cheap (cached), never raises."""
        return IDLE

    def program_events(self) -> list[ProgramEvent]:
        """Output lines / errors since the last call (drained)."""
        return []

    def program_finish(self) -> None:
        """After a run ended: restore what program_start changed (speed
        fraction, simulation mode, breakpoints). Best effort."""

    @abc.abstractmethod
    def set_motion_enabled(self, enabled: bool) -> None:
        """Enable (True) or disable (False) the motors. Enable also readies the
        arm (position mode, state ready); disable cuts motor power/torque."""

    @abc.abstractmethod
    def set_manual_mode(self, enabled: bool) -> None:
        """Enter (True) manual/free-drive mode so the arm can be hand-moved, or
        leave it (False), returning to position mode."""

    @abc.abstractmethod
    def go_home(self) -> None:
        """Controlled move to the home pose (asynchronous; does not block)."""

    @abc.abstractmethod
    def stop_motion(self) -> None:
        """Halt an in-flight controlled move (e.g. cancel a home move) and leave
        the controller position-READY."""

    @abc.abstractmethod
    def set_servo_cartesian(self, pose: Pose) -> int:
        """Command one Cartesian target (already safety-clamped). Returns the SDK
        code: 0 = accepted; nonzero = controller not ready (transient -> re-arm and
        retry, never a latched fault). Raises ArmFault only on a real failure."""

    @abc.abstractmethod
    def set_gripper(self, gripper_mm: float) -> None: ...

    @abc.abstractmethod
    def read_snapshot(self) -> ArmSnapshot: ...


class MockArmController(ArmController):
    """Deterministic in-memory arm for tests and the headless sim. It simply
    reports back whatever it was last commanded (the safety clamping upstream is
    what makes that motion bounded), so tests can assert on commanded poses
    without hardware. `moving` reflects whether the last command changed the pose.
    """

    def __init__(self, start_pose: Pose = (300.0, 0.0, 300.0, 180.0, 0.0, 0.0),
                 start_gripper: float = 84.0,
                 robot_model: str | None = "uf850") -> None:
        self._pose: Pose = start_pose
        self._gripper = start_gripper
        self._connected = False
        self._servo_active = False
        self._moving = False
        self._error_code = 0
        self._warn_code = 0
        self.command_count = 0  # test/telemetry aid
        # Joint angles to report back (deg, J1..J6). None = no joint stream (the
        # mock's default); tests/sim set it to exercise joint-aware paths such as
        # the servo loop's wind-up warning.
        self.joints: tuple[float, ...] | None = None
        # The mock has no identity of its own; it stands in for the UF850 by
        # default (its start pose is xArm-shaped). The config `robot_model` key
        # overrides this to preview another arm's model.
        self._robot_model = robot_model
        # Failure injection for tests: method name -> ArmFault code. A method in
        # this map raises (or, for clear_error, reports failure) so servo-loop
        # fault handling can be exercised without hardware.
        self._fail_methods: dict[str, int] = {}
        # Program stand-in: walks the program's lines on a timer, every
        # optional feature on (program_runtime.FakeLuaRuntime).
        self.runtime = FakeLuaRuntime()
        self.program_runs: list[dict] = []          # test aid: program_start args
        self.retreat_calls: list[tuple] = []        # test aid: retreat_to args
        self.end_stream_calls: list[str] = []       # test aid: end_stream reasons
        self.manual_mode = False
        # Power model (settled phase, power-up start, the power-up fails),
        # replaced as a whole so the status thread reads it without a lock.
        self._power: tuple[str, float | None, bool] = ("on", None, False)

    # ---- failure injection (tests) ---------------------------------------
    def fail_on(self, method: str, code: int = 1) -> None:
        self._fail_methods[method] = code

    def clear_failure(self, method: str) -> None:
        self._fail_methods.pop(method, None)

    def _maybe_fail(self, method: str) -> None:
        if method in self._fail_methods:
            raise ArmFault(f"injected fault in {method}", self._fail_methods[method])

    def connect(self) -> None:
        self._maybe_fail("connect")
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False
        self._servo_active = False
        self._moving = False

    def enable_servo(self) -> None:
        self._maybe_fail("enable_servo")
        self._servo_active = True

    def disable_servo(self) -> None:
        self._maybe_fail("disable_servo")
        self._servo_active = False
        self._moving = False

    def rearm(self) -> None:
        self._maybe_fail("rearm")
        self._servo_active = True   # re-armed -> drivable again

    def clear_error(self) -> tuple[bool, int, str]:
        # Truthful residual model: if a clear failure is injected, report the
        # residual code (ok=False) rather than raising -- mirrors a real clear
        # that ran but the physical cause persists.
        if "clear_error" in self._fail_methods:
            code = self._fail_methods["clear_error"]
            return (False, code, "clear failed: fault persists")
        self._error_code = 0
        self._warn_code = 0
        return (True, 0, "")

    def set_motion_enabled(self, enabled: bool) -> None:
        self._maybe_fail("set_motion_enabled")
        self._servo_active = enabled and self._servo_active
        self._moving = False
        if not enabled:
            self._power = ("off", None, False)
        elif self.power_info()["phase"] != "on":
            self._power = ("on", time.time(), "power_up" in self._fail_methods)

    # Power model for the dashboard -- reporting only, the mock still drives
    # while "off". A power-up walks powering -> releasing -> on, POWER_STEP_S
    # each; fail_on("power_up") makes the brake release time out instead.
    supports_power = True
    POWER_STEP_S = 1.0

    def power_info(self) -> dict | None:
        settled, started, fails = self._power
        phase, error = settled, None
        if started is not None:
            t = time.time() - started
            if t < self.POWER_STEP_S:
                phase = "powering"
            elif t < 2 * self.POWER_STEP_S:
                phase = "releasing"
            elif fails:
                phase, error = "idle", "brake release timed out (mock)"
        mode = {"off": "PowerOff", "idle": "Idle", "powering": "PowerOn",
                "releasing": "BrakeReleasing", "on": "Running"}[phase]
        return {"phase": phase, "robot_mode": mode, "safety_mode": "Normal",
                "busy": phase in ("powering", "releasing"), "error": error,
                "error_at_ms": (started + 2 * self.POWER_STEP_S) * 1000.0 if error else None}

    def set_manual_mode(self, enabled: bool) -> None:
        self._maybe_fail("set_manual_mode")
        self._servo_active = False
        self._moving = False
        self.manual_mode = enabled

    def go_home(self) -> None:
        self._maybe_fail("go_home")
        self._servo_active = False  # real path runs set_mode(0) before the move
        self._moving = True

    supports_retreat = True

    def end_stream(self, reason: str) -> None:
        self.end_stream_calls.append(reason)

    def retreat_to(self, joints_deg: list[float] | None, pose: Pose | None) -> None:
        # The mock "arrives" instantly: the retreat target becomes its readback
        # (joints and/or TCP), so the servo loop's posture gate sees it inside.
        self._maybe_fail("retreat_to")
        self.retreat_calls.append((joints_deg, pose))
        if joints_deg is not None:
            self.joints = tuple(float(v) for v in joints_deg)
        if pose is not None:
            self._pose = tuple(pose)
        self._servo_active = False
        self._moving = False        # arrived: the loop's stand-down ends after its grace

    def stop_motion(self) -> None:
        self._maybe_fail("stop_motion")
        self._moving = False        # motion halted; loop re-enters servo on reconcile

    def set_servo_cartesian(self, pose: Pose) -> int:
        self._maybe_fail("set_servo_cartesian")   # injected fault still raises
        self._moving = pose != self._pose
        self._pose = pose
        self.command_count += 1
        return 0

    def set_gripper(self, gripper_mm: float) -> None:
        self._maybe_fail("set_gripper")
        self._gripper = gripper_mm

    def read_snapshot(self) -> ArmSnapshot:
        if not self._connected:
            state = ArmState.UNKNOWN
        elif self._error_code != 0:
            state = ArmState.ERROR
        elif self._servo_active:
            state = ArmState.SERVO
        else:
            state = ArmState.DISABLED
        return ArmSnapshot(
            pose=self._pose, gripper=self._gripper, arm_state=state,
            moving=self._moving, servo_active=self._servo_active,
            protective_stop=False, error_code=self._error_code, warn_code=self._warn_code,
            joints=self.joints,
        )

    def robot_model(self) -> str | None:
        return self._robot_model

    # ---- programs: the fake runtime, every optional feature on ------------
    supports_programs = True

    def program_features(self) -> frozenset[str]:
        return frozenset(PROGRAM_FEATURES)

    def program_start(self, source: str, *, simulate: bool, speed_fraction: float,
                      breakpoints: tuple[int, ...] = (),
                      line_plan: tuple[int, ...] = ()) -> None:
        from .error_codes import NOT_READY, PROGRAM_RUNNING
        self._maybe_fail("program_start")
        if not self._connected:
            raise ArmFault("arm not connected", NOT_READY)
        try:
            self.runtime.start(source, line_plan, speed_fraction, breakpoints)
        except RuntimeError as exc:
            raise ArmFault(str(exc), PROGRAM_RUNNING) from exc
        self._servo_active = False
        self.program_runs.append({"source": source, "simulate": simulate,
                                  "speed_fraction": speed_fraction,
                                  "breakpoints": tuple(breakpoints),
                                  "line_plan": tuple(line_plan)})

    def program_control(self, op: str) -> None:
        from .error_codes import NOT_READY
        self._maybe_fail("program_control")
        try:
            self.runtime.control(op)
        except RuntimeError as exc:
            raise ArmFault(str(exc), NOT_READY) from exc

    def program_stop(self) -> None:
        self._maybe_fail("program_stop")
        self.runtime.stop()
        self._moving = False

    def program_set_breakpoints(self, lines: tuple[int, ...]) -> None:
        self.runtime.set_breakpoints(lines)

    def program_poll(self) -> ProgramPoll:
        return self.runtime.poll()

    def program_events(self) -> list[ProgramEvent]:
        return self.runtime.events()

    # test helpers
    @property
    def pose(self) -> Pose:
        return self._pose

    @property
    def gripper(self) -> float:
        return self._gripper

    def inject_error(self, code: int) -> None:
        self._error_code = code
