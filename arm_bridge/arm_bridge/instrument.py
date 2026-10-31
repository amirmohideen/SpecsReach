"""`robot`-source logging at the ArmController boundary (SIM-DASHBOARD-PLAN §4).

`LoggingArmController` wraps any `ArmController` (mock/sim/xarm) so every
backend gets the same observability for free. It is a pure pass-through --
same calls, same returns, same exceptions -- with taps that publish LogEvents
to the bus.

Volume discipline (plan review): the servo/status threads call this wrapper at
up to servo_hz + status_hz per second, so per-call logging of the hot paths
would drown the bus (a 10k ring ≈ one minute). Hot paths are therefore
edge-logged:
- `set_servo_cartesian` / `set_gripper`: 1 Hz summaries (count + last value)
  plus immediate-but-throttled nonzero-code warnings.
- `read_snapshot`: never logged per call; only *transitions* of
  (arm_state, error_code, warn_code, protective_stop) are published.
- One-shot command calls (connect/enable/home/...) are always logged with
  latency; `ArmFault` is published at error level and re-raised unchanged.
"""

from __future__ import annotations

import time

from .arm_interface import ArmController, ArmFault, ArmSnapshot, Pose
from .logbus import LogBus

SUMMARY_PERIOD_S = 1.0


class LoggingArmController(ArmController):
    def __init__(self, inner: ArmController, bus: LogBus,
                 clock=time.monotonic) -> None:
        self.inner = inner
        self._bus = bus
        self._clock = clock
        # set_servo_cartesian summary window
        self._servo_count = 0
        self._servo_nonzero = 0
        self._servo_last: Pose | None = None
        self._servo_summary_at = 0.0
        self._nonzero_warn_at = -1e9
        self._nonzero_suppressed = 0
        # set_gripper summary window
        self._grip_count = 0
        self._grip_last: float | None = None
        self._grip_summary_at = 0.0
        # read_snapshot transition key
        self._snap_key: tuple | None = None

    # ---- helpers ----------------------------------------------------------
    def _publish(self, level: str, text: str, data: dict | None = None) -> None:
        self._bus.publish("robot", level, text, data)

    def _call(self, name: str, fn, *args, detail: str = "", **kwargs):
        """One-shot command tap: log call + latency; ArmFault -> error event,
        re-raised unchanged (the servo loop owns fault latching)."""
        t0 = self._clock()
        try:
            result = fn(*args, **kwargs)
        except ArmFault as exc:
            self._publish("error",
                          f"{name}{detail} raised ArmFault code {exc.code}: {exc.message}",
                          {"call": name, "code": exc.code, "ms": self._ms(t0)})
            raise
        self._publish("info", f"{name}{detail} ok",
                      {"call": name, "ms": self._ms(t0)})
        return result

    def _ms(self, t0: float) -> float:
        return round((self._clock() - t0) * 1000.0, 2)

    # ---- one-shot command calls -------------------------------------------
    def connect(self) -> None:
        self._call("connect", self.inner.connect)

    def disconnect(self) -> None:
        self._call("disconnect", self.inner.disconnect)

    def enable_servo(self) -> None:
        self._call("enable_servo", self.inner.enable_servo)

    def disable_servo(self) -> None:
        self._call("disable_servo", self.inner.disable_servo)

    def rearm(self) -> None:
        self._call("rearm", self.inner.rearm)

    def clear_error(self) -> tuple[bool, int, str]:
        t0 = self._clock()
        try:
            ok, code, msg = self.inner.clear_error()
        except ArmFault as exc:
            self._publish("error",
                          f"clear_error raised ArmFault code {exc.code}: {exc.message}",
                          {"call": "clear_error", "code": exc.code, "ms": self._ms(t0)})
            raise
        level = "info" if ok else "warn"
        self._publish(level, f"clear_error -> ok={ok} code={code} {msg}".rstrip(),
                      {"call": "clear_error", "ok": ok, "code": code, "ms": self._ms(t0)})
        return ok, code, msg

    def set_motion_enabled(self, enabled: bool) -> None:
        self._call("set_motion_enabled", self.inner.set_motion_enabled, enabled,
                   detail=f"({enabled})")

    def set_manual_mode(self, enabled: bool) -> None:
        self._call("set_manual_mode", self.inner.set_manual_mode, enabled,
                   detail=f"({enabled})")

    def go_home(self) -> None:
        self._call("go_home", self.inner.go_home)

    def stop_motion(self) -> None:
        self._call("stop_motion", self.inner.stop_motion)

    # ---- hot paths (edge-logged) ------------------------------------------
    def set_servo_cartesian(self, pose: Pose) -> int:
        t0 = self._clock()
        try:
            code = self.inner.set_servo_cartesian(pose)
        except ArmFault as exc:
            self._publish("error",
                          f"set_servo_cartesian raised ArmFault code {exc.code}: {exc.message}",
                          {"call": "set_servo_cartesian", "code": exc.code,
                           "pose": [round(v, 2) for v in pose], "ms": self._ms(t0)})
            raise
        now = self._clock()
        self._servo_count += 1
        self._servo_last = pose
        if code:
            self._servo_nonzero += 1
            # A persistent not-ready reply arrives at loop rate; throttle to
            # ~1 Hz with a suppressed count so the storm is visible, not deafening.
            if now >= self._nonzero_warn_at:
                suffix = (f" ({self._nonzero_suppressed} suppressed)"
                          if self._nonzero_suppressed else "")
                self._publish("warn",
                              f"set_position code {code} (controller not ready){suffix}",
                              {"call": "set_servo_cartesian", "code": code})
                self._nonzero_warn_at = now + SUMMARY_PERIOD_S
                self._nonzero_suppressed = 0
            else:
                self._nonzero_suppressed += 1
        if now >= self._servo_summary_at and self._servo_count:
            # Full six components: x/y/z alone made a faulting orientation
            # (e.g. controller error 23) unreconstructable from the journal.
            self._publish("debug",
                          f"set_position x{self._servo_count} last "
                          f"({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f} / "
                          f"{pose[3]:.1f}, {pose[4]:.1f}, {pose[5]:.1f}) "
                          f"nonzero={self._servo_nonzero}",
                          {"call": "set_servo_cartesian", "count": self._servo_count,
                           "nonzero": self._servo_nonzero,
                           "pose": [round(v, 2) for v in pose]})
            self._servo_summary_at = now + SUMMARY_PERIOD_S
            self._servo_count = 0
            self._servo_nonzero = 0
        return code

    def set_gripper(self, gripper_mm: float) -> None:
        t0 = self._clock()
        try:
            self.inner.set_gripper(gripper_mm)
        except ArmFault as exc:
            self._publish("error",
                          f"set_gripper raised ArmFault code {exc.code}: {exc.message}",
                          {"call": "set_gripper", "code": exc.code, "ms": self._ms(t0)})
            raise
        now = self._clock()
        self._grip_count += 1
        changed = self._grip_last is None or abs(gripper_mm - self._grip_last) > 0.5
        self._grip_last = gripper_mm
        if changed and now >= self._grip_summary_at:
            self._publish("debug",
                          f"set_gripper x{self._grip_count} last {gripper_mm:.1f} mm",
                          {"call": "set_gripper", "count": self._grip_count,
                           "mm": round(gripper_mm, 1)})
            self._grip_summary_at = now + SUMMARY_PERIOD_S
            self._grip_count = 0

    def read_snapshot(self) -> ArmSnapshot:
        snap = self.inner.read_snapshot()
        key = (snap.arm_state, snap.error_code, snap.warn_code, snap.protective_stop)
        if key != self._snap_key:
            prev = self._snap_key
            self._snap_key = key
            level = "error" if (snap.error_code or snap.protective_stop) \
                else ("warn" if snap.warn_code else "info")
            # Joint posture on every transition: limit faults (error 23) are
            # invisible in the 3D view (J1/J4/J6 wrap mod 360; J3's limit is a
            # straight elbow), so the numbers must be in the journal.
            joints = ("[" + ", ".join(f"{j:.1f}" for j in snap.joints) + "]"
                      if snap.joints else None)
            self._publish(level,
                          f"report: {snap.arm_state.name} err={snap.error_code} "
                          f"warn={snap.warn_code} pstop={snap.protective_stop}"
                          + (f" (was {prev[0].name} err={prev[1]})" if prev else "")
                          + (f" joints={joints}" if joints else ""),
                          {"call": "read_snapshot", "state": snap.arm_state.name,
                           "error_code": snap.error_code, "warn_code": snap.warn_code,
                           "joints": [round(j, 2) for j in snap.joints]
                           if snap.joints else None})
        return snap

    # Explicit: ArmController defines a default for each of these, so
    # __getattr__ never fires for them and the wrapper would silently answer
    # with the base-class default instead of the backend's.
    def robot_model(self) -> str | None:
        return self.inner.robot_model()

    def read_fault_detail(self) -> str:
        # Servo-thread fault path only (servo-motor faults): the backend's
        # per-joint detail must reach the journal through the wrapper.
        return self.inner.read_fault_detail()

    @property
    def streams_every_tick(self) -> bool:
        return self.inner.streams_every_tick

    @property
    def supports_set_home(self) -> bool:
        return self.inner.supports_set_home

    @property
    def supports_retreat(self) -> bool:
        return self.inner.supports_retreat

    @property
    def supports_power(self) -> bool:
        return self.inner.supports_power

    @property
    def supports_joint_replay(self) -> bool:
        return self.inner.supports_joint_replay

    def joint_replay_limits_deg(self) -> tuple[float, float]:
        return self.inner.joint_replay_limits_deg()

    def set_servo_joints(self, joints_deg) -> int:
        code = self.inner.set_servo_joints(joints_deg)  # per tick: logged only when refused
        if code not in (0, 1):
            self._publish("warn", f"set_servo_joints refused: code {code}",
                          {"call": "set_servo_joints", "code": code,
                           "joints": [round(v, 2) for v in joints_deg]})
        return code

    def power_info(self) -> dict | None:
        return self.inner.power_info()     # status thread, every publish: never logged

    def retreat_to(self, joints_deg, pose) -> None:
        detail = (f"(joints {[round(v, 1) for v in joints_deg]})" if joints_deg is not None
                  else f"(pose {[round(v, 1) for v in pose]})" if pose is not None else "()")
        self._call("retreat_to", self.inner.retreat_to, joints_deg, pose, detail=detail)

    def joint_limits_deg(self) -> list[tuple[float, float]] | None:
        return self.inner.joint_limits_deg()

    def service(self, now: float) -> None:
        self.inner.service(now)        # per tick: never logged

    def journal_fields(self) -> dict:
        return self.inner.journal_fields()

    def end_stream(self, reason: str) -> None:
        # Explicit: the base-class no-op would shadow __getattr__ delegation.
        self.inner.end_stream(reason)

    def set_home_here(self) -> tuple[bool, int, str]:
        t0 = self._clock()
        ok, code, msg = self.inner.set_home_here()
        self._publish("info" if ok else "warn",
                      f"set_home_here -> ok={ok} code={code} {msg}".rstrip(),
                      {"call": "set_home_here", "ok": ok, "code": code,
                       "ms": self._ms(t0)})
        return ok, code, msg

    # Programs: explicit for the same reason (base-class defaults). Commands
    # are logged; poll/events run every tick and are not.
    @property
    def supports_programs(self) -> bool:
        return self.inner.supports_programs

    def program_features(self) -> frozenset[str]:
        return self.inner.program_features()

    def program_start(self, source: str, *, simulate: bool, speed_fraction: float,
                      breakpoints: tuple[int, ...] = (),
                      line_plan: tuple[int, ...] = ()) -> None:
        self._call("program_start", self.inner.program_start, source,
                   simulate=simulate, speed_fraction=speed_fraction,
                   breakpoints=breakpoints, line_plan=line_plan,
                   detail=f"({len(source)} chars, {'sim' if simulate else 'real'}, "
                          f"speed {speed_fraction:.2f})")

    def program_control(self, op: str) -> None:
        self._call("program_control", self.inner.program_control, op, detail=f"({op})")

    def program_stop(self) -> None:
        self._call("program_stop", self.inner.program_stop)

    def program_set_breakpoints(self, lines: tuple[int, ...]) -> None:
        self._call("program_set_breakpoints", self.inner.program_set_breakpoints, lines,
                   detail=f"({list(lines)})")

    def program_poll(self):
        return self.inner.program_poll()

    def program_events(self):
        return self.inner.program_events()

    def program_finish(self) -> None:
        self._call("program_finish", self.inner.program_finish)

    # Mock/test helpers (fail_on, inject_error, pose, ...) stay reachable
    # through the wrapper so the sim/tests don't need to unwrap it.
    def __getattr__(self, name: str):
        return getattr(self.inner, name)
