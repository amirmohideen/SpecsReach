"""The servo loop -- the only thing that talks to the arm.

Each tick it reads a DrivePlan from the Session and reconciles the arm to it:
enter/leave mode-7 online-planning control, run pending CLEAR_ERROR/ARM_ACTION,
re-arm when the controller idles, and (when driving) stream the latest Cartesian
target via set_position on-change. When not driving -- deadman released, watchdog
expired, or SET_ENABLED(0) -- it stops streaming; mode 7 holds position itself.

`tick()` is a pure step over an injected clock so the whole loop is unit-testable
without real time; `run()` wraps it in a paced thread for production.

Station control mode -- who may command the arm, one owner at a time:
  teleop      the Quest drives (the default, and after every restart);
  free_drive  the arm is hand-guided (the manual latch; the Quest's MANUAL
              on/off flips the same switch);
  program     a controller program may run (ProgramSupervisor); the loop
              streams nothing, Quest motion actions are refused and the Quest
              sees HOMING while a program runs (its Cancel = stop).
Mode changes come from the dashboard as LOCAL_ACTION_MODE_* (refused while
the Quest is clutched in, or a program runs). `request_stop()` is the
universal Stop, open to everyone: hold teleop (the Quest must re-clutch), end
free-drive, abort a program -- whatever brings the arm to rest.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque

from . import error_codes, safety
from .journal import MotionJournal
from .arm_interface import ArmController, ArmFault, ArmSnapshot
from .config import ArmBridgeConfig
from .protocol import (
    ARM_ACTION_AUTOREARM_OFF,
    ARM_ACTION_AUTOREARM_ON,
    ARM_ACTION_BACKOUT,
    ARM_ACTION_HOME,
    ARM_ACTION_HOME_CANCEL,
    ARM_ACTION_MANUAL_OFF,
    ARM_ACTION_MANUAL_ON,
    ARM_ACTION_MOTION_DISABLE,
    ARM_ACTION_MOTION_ENABLE,
    ARM_ACTION_REARM,
    ARM_ACTION_RECOVER,
    ARM_ACTION_RETREAT,
    ArmState,
    Status,
)
from .program_supervisor import PendingResult
from .session import (
    LOCAL_ACTION_HOME,
    LOCAL_ACTION_RECOVER,
    LOCAL_ACTION_MODE_FREE_DRIVE,
    LOCAL_ACTION_MODE_PROGRAM,
    LOCAL_ACTION_MODE_TELEOP,
    LOCAL_ACTION_POWER_OFF,
    LOCAL_ACTION_POWER_ON,
    LOCAL_ACTION_RETREAT,
    LOCAL_ACTION_SET_HOME,
    ControlResult,
    Session,
)

Pose = tuple[float, float, float, float, float, float]

log = logging.getLogger("arm_bridge.servo")

MODES = ("teleop", "free_drive", "program")
_MODE_ACTIONS = {LOCAL_ACTION_MODE_TELEOP: "teleop",
                 LOCAL_ACTION_MODE_FREE_DRIVE: "free_drive",
                 LOCAL_ACTION_MODE_PROGRAM: "program"}
# Arm actions still accepted in Program mode: power, recovery, stop and pure
# policy flags. Everything that moves the arm or changes the mode is refused.
# Dashboard actions that ARE a wire action (same path, same bookkeeping).
_LOCAL_WIRE = {LOCAL_ACTION_HOME: ARM_ACTION_HOME, LOCAL_ACTION_RECOVER: ARM_ACTION_RECOVER,
               LOCAL_ACTION_POWER_ON: ARM_ACTION_MOTION_ENABLE,
               LOCAL_ACTION_POWER_OFF: ARM_ACTION_MOTION_DISABLE,
               LOCAL_ACTION_RETREAT: ARM_ACTION_RETREAT}
_PROGRAM_MODE_ACTIONS = frozenset({
    ARM_ACTION_MOTION_ENABLE, ARM_ACTION_MOTION_DISABLE, ARM_ACTION_RECOVER,
    ARM_ACTION_HOME_CANCEL, ARM_ACTION_AUTOREARM_ON, ARM_ACTION_AUTOREARM_OFF,
    LOCAL_ACTION_SET_HOME})


def _fmt_pose(pose) -> str:
    if pose is None:
        return "none"
    return "(" + ", ".join(f"{v:.1f}" for v in pose) + ")"


def _fmt_joints(joints) -> str:
    if not joints:
        return "none"
    return "[" + ", ".join(f"{v:.1f}" for v in joints) + "]"

# Send a new set_position only when the target moved this much since the last one
# -- mode 7 re-plans on each call, so streaming identical targets at loop rate is
# wasteful and can choke the planner (the working reference sends on-change).
MIN_SEND_MM = 0.2
MIN_SEND_DEG = 0.1
# Min gap between automatic re-arms (the controller idles to state 5 MODE_CHANGED;
# re-arming is set_state 0). Rate-limited so a persistent non-ready never storms.
REARM_COOLDOWN_S = 1.0
# After re-asserting mode 7 + state 0 (enable_servo / rearm), the controller needs
# a moment to become "ready to move" before it will accept set_position. Streaming
# a target during this window drew a code-1 ("not ready to move") reply AND
# controller error 35. We hold off the first send by SETTLE_S instead of sleeping
# (the loop must keep running for the watchdog). The reference sleeps 0.3 s once at
# startup, but its in-loop set_state(0) -> set_position gap is ~one tick (20 ms);
# 100 ms is a safe margin over that and stays under the watchdog so a held clutch
# never starves during the settle.
SETTLE_S = 0.10
# Motion journal: auto-dump this long after an incident, at most once per gap.
AUTO_DUMP_DELAY_S = 0.5
AUTO_DUMP_GAP_S = 10.0
# BACKOUT trail: a rolling window of the arm's ACTUAL recent path (pose and,
# when the backend reports them, joints), recorded every healthy tick and FROZEN
# the moment the loop stops reading (fault, manual, home standdown) -- the
# operator may press BACK OUT seconds after the fault, and the pre-fault trail
# must still be there. Points are deduped by motion (pose OR any joint), so a
# stationary arm -- e.g. one standing on a "target held" warning -- does not
# fill the window with copies or age the last motion out of it.
TRAIL_WINDOW_S = 3.0
TRAIL_MIN_STEP_MM = 0.5
TRAIL_MIN_STEP_DEG = 0.5
# A joint replay retries a not-yet-sent point (trajectory path being cleared,
# smooth stop settling) at most this long before giving up.
BACKOUT_WAIT_S = 2.0
# Controller error codes that are e-stops: BACKOUT degrades to clear-only for
# these -- automatic motion straight after an e-stop release is never right.
ESTOP_ERROR_CODES = frozenset((1, 2, 3))

# Wrist/base wind-up warning. J1/J4/J6 have +/-360 deg limits, so mode-7 IK can
# wind one toward its limit while the commanded RPY stays in (-180, 180] and the
# 3D viewer (rotation is mod 360) shows nothing amiss -- then the controller
# faults (error 23) at what looks like an innocent pose. Warn past this
# magnitude so the operator can HOME/unwind first. J2/J3/J5 cannot wrap; the
# dashboard's per-joint "hot" bar covers those. Hysteresis so a joint hovering
# at the edge does not re-warn every status tick.
WINDUP_WARN_DEG = 300.0
WINDUP_CLEAR_DEG = 280.0
WRAP_JOINT_INDEXES = (0, 3, 5)   # J1, J4, J6 (the +/-360 deg joints)


class ServoLoop:
    def __init__(self, config: ArmBridgeConfig, session: Session,
                 arm: ArmController, clock=time.monotonic, programs=None,
                 wall_clock=time.time, capture_tap=None,
                 capture_snapshots: bool = True) -> None:
        self._cfg = config
        self._session = session
        self._arm = arm
        self._clock = clock
        # Data capture (capture_tap.py): each tick's command, and -- for
        # backends without their own state stream -- each STATUS snapshot.
        # Hand-off only; never touches motion.
        self._capture = capture_tap
        self._capture_snapshots = capture_snapshots
        self._wall_clock = wall_clock
        # Program runs (dashboard + a backend with supports_programs); None =
        # Program mode unavailable.
        self._programs = programs
        self._program_mode = False
        self._mode_info = {"mode": "teleop", "set_by": "start",
                           "set_at_ms": wall_clock() * 1000.0}
        # Universal Stop requests from any thread, handled first thing in tick.
        self._stop_lock = threading.Lock()
        self._stop_requests: list[tuple[str, PendingResult]] = []
        # Backend streams its target every driving tick (it rate-limits toward
        # the target itself) instead of the default send-on-change gate.
        self._stream_every_tick = bool(getattr(arm, "streams_every_tick", False))
        self._control_on = False          # in mode-7 control (vs. plan.want_servo)
        self._last_sent_pose: Pose | None = None  # last set_position target (send-on-change)
        self._was_driving = False
        # Auto re-arm: when True (default) the loop re-arms the controller itself
        # when it idles to a re-armable state (mode-7 MODE_CHANGED). When the
        # operator turns it OFF (Arm tab), the loop holds instead and waits for a
        # manual RESUME (ARM_ACTION_REARM), so they control reset/resume.
        self._auto_rearm = True
        self._rearm_at = -1e9              # so the first re-arm is never cooldown-blocked
        # Re-arm-before-drive gate. Set whenever the controller may have idled to a
        # not-ready state (start of a drive burst, after a not-ready set_position
        # reply, after the deadman released): the loop re-asserts state 0 and lets
        # it settle BEFORE streaming, so it never commands a not-ready controller
        # (the cause of "not ready to move" + error 35).
        self._need_rearm = True
        # No set_position before this time -- the post-arm settle window (SETTLE_S).
        self._ready_at = 0.0
        # Manual/free-drive latch. While set, the loop must not touch the arm:
        # the normal reconcile (set_mode(7)) would stomp teach mode 2. Without
        # this latch MANUAL_ON was immediately undone by the next tick, forcing
        # the operator through a set/unset/set dance. Cleared by MANUAL_OFF or
        # any other action that re-enters position mode (home/enable/recover).
        self._manual_active = False
        # HOME standdown: move_gohome is an asynchronous mode-0 joint move, and
        # an xArm MODE SWITCH TERMINATES THE EXECUTING MOTION -- so if the
        # reconcile re-entered control (set_mode(7)) on the next tick, the home
        # move died ~4 ms in and the hold-stream then pinned the arm at its
        # current pose ("HOME ok" reply, zero movement; Studio worked because
        # nothing flipped the mode behind its back). While set, the loop stands
        # down; released once the grace period has passed AND the arm reports
        # not-moving (the move finished), or by any subsequent action.
        self._home_until_idle = False
        self._home_action_at = 0.0
        # Envelope gate: why the arm's ACTUAL posture is outside the configured
        # envelope (bridge joint limits, box, reach, keep-outs), or None. Set
        # when leaving free-drive and re-checked at every engage / burst start;
        # while set the loop never enters control (STATUS warn 1006) and the
        # dashboard offers RETREAT. Servo-thread write, GIL-atomic reads.
        self._envelope_block: str | None = None
        self._retreat_moving = False       # the last RETREAT started a move
        # Latched actuator fault (rev 3 sec 10 / safety review): a raising SDK call
        # must never escape and kill this thread. When faulted we hold and keep
        # STATUS alive (=ERROR) until a CLEAR_ERROR succeeds. Written on the servo
        # thread, read on the status thread -> guarded by _fault_lock.
        self._fault_lock = threading.Lock()
        self._faulted = False
        self._fault_code = 0
        # The ArmFault message that latched the fault. Retained for the journal
        # and diagnosis -- the code alone is ambiguous (SDK API codes and
        # controller error codes collide numerically; see STATUS_FLAG_ERRCODE_IS_API).
        self._fault_msg = ""
        # BACKOUT (operator-triggered undo): rolling trail of the arm's actual
        # recent poses, and the reversed waypoint queue being replayed. Both are
        # servo-thread state; status() only reads len() (GIL-atomic, like the
        # home standdown flag).
        # Trail entries: (time, pose, joints deg or None).
        self._trail: deque[tuple[float, tuple, tuple | None]] = deque()
        self._backout_queue: list = []
        self._backout_joints = False   # the queue holds joint points (deg), not poses
        self._backout_wait_since: float | None = None
        # Last read_snapshot() taken by status(), for the dashboard StateHub
        # (carries joints, which the wire Status does not).
        self.last_snapshot: ArmSnapshot | None = None
        # Joint indexes currently past the wind-up warn threshold (edge-tracked so
        # the warning fires once per excursion, not every status tick). Touched
        # only on the arm-status thread via status()/_check_windup.
        self._windup_warned: set[int] = set()
        # Last commanded drive target, for the dashboard's ghost triad; None
        # while holding. Plain ref writes on the servo thread, GIL-atomic reads.
        self.last_target: tuple | None = None
        # Motion journal (B5): one row per tick, events, auto-dump on faults.
        self.journal = MotionJournal(
            int(config.journal_seconds * config.servo_hz),
            meta={"backend": config.arm_backend, "robot_model": config.robot_model,
                  "servo_hz": config.servo_hz,
                  "velocity": [config.velocity.max_linear_mm_s, config.velocity.max_angular_deg_s,
                               config.velocity.max_linear_acc_mm_s2,
                               config.velocity.max_angular_acc_deg_s2],
                  "safety_factor": config.safety_factor},
            wall_clock=wall_clock)
        self._jr_plan = None
        self._jr_snap: ArmSnapshot | None = None
        self._jr_driving = False
        self._jr_state: str | None = None
        self._dump_at: float | None = None
        self._dump_reason = ""
        self._last_auto_dump = -1e9
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def _enter_fault(self, code: int, message: str = "") -> None:
        """Latch ERROR and drop control bookkeeping; best-effort hold. Recovery
        is the operator's explicit CLEAR_ERROR/RECOVER -- we never auto-clear (an
        earlier auto-recovery turned one fault into a clear/re-enter/re-fault
        storm)."""
        with self._fault_lock:
            self._faulted = True
            self._fault_code = code
            self._fault_msg = message
        now = self._clock()
        self.journal.event(now, "fault", code=code, msg=message)
        self._schedule_dump(now, f"fault-{code}")
        # Forensics: the commanded pose stream logs only x/y/z at 1 Hz, so the
        # faulting orientation and joint posture are unrecoverable unless dumped
        # here (the error-23 incident, 2026-09-06). last_snapshot is the status
        # thread's read -- never re-read the arm from a fault path.
        snap = self.last_snapshot
        log.error("arm: FAULT latched code=%s (%s) last_sent=%s target=%s joints=%s",
                  code, message or "no detail",
                  _fmt_pose(self._last_sent_pose), _fmt_pose(self.last_target),
                  _fmt_joints(snap.joints if snap is not None else None))
        # Servo-motor errors (controller codes 10-17, e.g. E14 joint 4) recur and
        # persist through a software clear -- power-cycle territory. The controller
        # code alone does not say WHY the joint tripped; pull the per-joint servo
        # sub-code once, here, so a recurring hardware fault is diagnosable from the
        # journal instead of guesswork. Servo thread owns arm access; best-effort.
        if 10 <= code <= 17:
            try:
                detail = self._arm.read_fault_detail()
            except Exception as exc:  # noqa: BLE001 -- must never escape the fault path
                detail = f"detail probe raised: {exc!r}"
            if detail:
                log.error("arm: FAULT servo detail code=%s %s", code, detail)
        self._control_on = False
        self._was_driving = False
        self._last_sent_pose = None
        self._need_rearm = True
        self._backout_queue = []    # a new fault voids any in-flight back-out
        try:
            self._arm.disable_servo()   # best effort; ignore a fault holding a fault
        except ArmFault:
            pass

    @property
    def _is_faulted(self) -> bool:
        with self._fault_lock:
            return self._faulted

    # ---- station control mode + universal Stop ---------------------------
    @property
    def mode(self) -> str:
        if self._program_mode:
            return "program"
        return "free_drive" if self._manual_active else "teleop"

    def mode_info(self) -> dict:
        """{mode, set_by, set_at_ms} for the dashboard (GIL-atomic read)."""
        return self._mode_info

    def available_modes(self) -> list[str]:
        modes = ["teleop", "free_drive"]
        if self._programs is not None and getattr(self._arm, "supports_programs", False):
            modes.append("program")
        return modes

    @property
    def programs(self):
        return self._programs

    def request_stop(self, source: str) -> PendingResult:
        """Universal Stop (any thread, never refused). In teleop the hold
        applies at once (the Quest must re-clutch); the rest runs on the servo
        thread next tick. Other modes leave the teleop latch alone -- the Quest
        cannot drive there, and a stale LATCHED would only mislead."""
        if self.mode == "teleop":
            self._session.latch_hold()
        pending = PendingResult()
        with self._stop_lock:
            self._stop_requests.append((source, pending))
        return pending

    def _note_mode(self, now: float, set_by: str) -> None:
        mode = self.mode
        if mode != self._mode_info["mode"]:
            self._mode_info = {"mode": mode, "set_by": set_by,
                               "set_at_ms": self._wall_clock() * 1000.0}
            log.info("mode: %s (set by %s)", mode, set_by)

    def _schedule_dump(self, now: float, reason: str) -> None:
        """Write the journal shortly after an incident (so the aftermath is in
        it), at most once per AUTO_DUMP_GAP_S."""
        if not self.journal.enabled or now - self._last_auto_dump < AUTO_DUMP_GAP_S:
            return
        self._last_auto_dump = now
        self._dump_at = now + AUTO_DUMP_DELAY_S
        self._dump_reason = reason

    def dump_journal(self, reason: str = "on demand") -> tuple[str | None, int]:
        """Write the journal now (dashboard thread)."""
        return self.journal.dump(self._cfg.journal_dir, reason)

    def _journal_tick(self, now: float) -> None:
        if not self.journal.enabled:
            return
        try:
            plan = self._jr_plan
            snap = self._jr_snap if self._jr_snap is not None else self.last_snapshot
            driving = bool(plan is not None and plan.drive)
            if driving and not self._jr_driving:
                self.journal.event(now, "engage")
            self._jr_driving = driving
            state = snap.arm_state.name if snap is not None else None
            if state != self._jr_state:
                if state in ("PROTECTIVE_STOP", "ERROR"):
                    self.journal.event(now, "state", state=state,
                                       err=snap.error_code if snap is not None else 0)
                    self._schedule_dump(now, state.lower())
                self._jr_state = state
            row = {"t": now, "drive": driving, "dm": self._session.deadman_held,
                   "tgt": plan.target_pose if plan is not None else None,
                   "mode": self.mode}
            if snap is not None:
                row.update(act=snap.pose, q=snap.joints, st=state,
                           err=snap.error_code, warn=snap.warn_code)
            row.update(self._arm.journal_fields())
            self.journal.record(row)
            if self._dump_at is not None and now >= self._dump_at:
                self._dump_at = None
                self.journal.dump_async(self._cfg.journal_dir, self._dump_reason)
        except Exception as exc:  # noqa: BLE001 -- the journal must never stop the loop
            log.debug("journal: tick record failed: %r", exc)

    def _burst_end_reason(self, now: float) -> str:
        """Why a drive burst ended (for end_stream / the journal)."""
        if not self._session.deadman_held:
            return "deadman released"
        if self._session.hold_latched:
            return "hold latched"
        if self._session.watchdog_expired(now):
            return "watchdog"
        return "no fresh target"

    def _release_control(self) -> None:
        """Leave teleop control bookkeeping (entering free-drive / program)."""
        if self._control_on:
            try:
                self._arm.disable_servo()
            except ArmFault as exc:
                log.warning("mode: disable_servo failed (%s)", exc.message)
        self._control_on = False
        self._was_driving = False
        self._last_sent_pose = None
        self.last_target = None
        self._need_rearm = True

    def _apply_mode(self, target: str, now: float) -> tuple[bool, int, str]:
        current = self.mode
        if target == current:
            return (True, 0, f"already in {target}")
        if target == "program" and "program" not in self.available_modes():
            return (False, error_codes.UNSUPPORTED,
                    "this arm backend (or a bridge without the dashboard) cannot run programs")
        if current == "program" and self._programs is not None and self._programs.busy:
            return (False, error_codes.PROGRAM_RUNNING,
                    "a program is running -- stop it first")
        if target != "teleop" and (self._home_until_idle or self._backout_queue):
            return (False, error_codes.NOT_READY,
                    "a HOME / back-out move is in flight -- wait for it or press Stop")
        if target == "free_drive":
            # Enter first: a failed enter leaves the mode unchanged.
            self._arm.set_manual_mode(True)
            self._program_mode = False
            self._manual_active = True
            self._home_until_idle = False
            self._release_control()
            self._trail.clear()          # hand-moved: the trail no longer applies
            return (True, 0, "free-drive on")
        if current == "free_drive":
            try:
                self._arm.set_manual_mode(False)
            except ArmFault as exc:
                if target == "program":
                    return (False, exc.code, f"could not leave free-drive: {exc.message}")
                # -> teleop: release the latch anyway (same rule as MANUAL_OFF:
                # never park the loop in a free-drive the client no longer shows).
                self._manual_active = False
                self._release_control()
                return (False, exc.code, f"free-drive off failed: {exc.message}")
            self._manual_active = False
        if target == "program":
            self._release_control()
            self._program_mode = True
            return (True, 0, "program mode: the Quest cannot drive")
        self._program_mode = False
        self._release_control()
        # The arm may have been hand-guided outside the envelope: say so now
        # (the engage gate below holds teleop until RETREAT / free-drive fixes it).
        reason = self._refresh_envelope()
        if reason:
            return (True, 0, f"teleop, but the arm is outside the envelope ({reason}) -- "
                             "RETREAT or hand-guide it back before driving")
        return (True, 0, "teleop: the Quest may drive")

    # ---- envelope gate (actual posture vs bridge limits) -----------------
    @property
    def envelope_block(self) -> str | None:
        """Why teleop is held by the arm's actual posture, or None (any thread)."""
        return self._envelope_block

    def _posture_violation(self, snap: ArmSnapshot | None) -> str | None:
        if snap is None or snap.arm_state == ArmState.UNKNOWN:
            return None                    # nothing to judge; the health gate holds anyway
        try:
            limits = self._arm.joint_limits_deg()
        except Exception:  # noqa: BLE001 -- a display helper must never block the gate
            limits = None
        return safety.posture_violation(snap.pose, snap.joints, limits, self._cfg.workspace)

    def _refresh_envelope(self, snap: ArmSnapshot | None = None) -> str | None:
        """Re-evaluate the envelope gate from a fresh (or given) snapshot;
        logs the edges so the journal shows when the arm left / re-entered."""
        if snap is None:
            try:
                snap = self._arm.read_snapshot()
            except ArmFault:
                return self._envelope_block
        reason = self._posture_violation(snap)
        if reason != self._envelope_block:
            if reason:
                log.warning("arm: outside the envelope -- %s; teleop held (RETREAT or "
                            "hand-guide the arm back)", reason)
            else:
                log.info("arm: back inside the envelope; teleop may engage")
            self._envelope_block = reason
        return reason

    def _take_stops(self) -> list[tuple[str, PendingResult]]:
        with self._stop_lock:
            stops, self._stop_requests = self._stop_requests, []
        return stops

    def _do_stop(self, now: float, source: str) -> str:
        parts = []
        if self._programs is not None and self._programs.busy:
            parts.append(self._programs.stop_now(now, source))
        if self._home_until_idle or self._backout_queue:
            try:
                self._arm.stop_motion()
            except ArmFault as exc:
                log.error("STOP: stop_motion failed: %s", exc.message)
            self._home_until_idle = False
            self._backout_queue = []
            parts.append("stopped the HOME / back-out move")
        if self._manual_active:
            try:
                self._arm.set_manual_mode(False)
            except ArmFault as exc:
                parts.append(f"free-drive off failed: {exc.message}")
            self._manual_active = False
            self._release_control()
            parts.append("ended free-drive")
        if self.mode == "teleop" and not parts:
            self._session.latch_hold()
            if self._control_on or self._was_driving:
                self._release_control()
            parts.append("teleop held: the Quest must release and re-engage the clutch")
        summary = "; ".join(parts) or "nothing to stop"
        log.warning("STOP (%s): %s", source, summary)
        self._note_mode(now, "stop")
        return summary

    def _program_blocks_clear(self, now: float, source: str) -> tuple[bool, int, str] | None:
        """A fault clear (CLEAR_ERROR, RECOVER, BACKOUT) while a controller
        program is active -- ours, or the pendant's -- must never let that
        program continue once the protective stop is unlocked. Abort it here,
        on the servo thread, so EVERY clear path gets the rule (the dashboard's
        Clear waits for the abort first; the Quest's clear is refused and asks
        for a second press once the program has stopped)."""
        if self._programs is None or not self._programs.busy:
            return None
        summary = self._programs.stop_now(now, f"{source} (clear fault)")
        return (False, error_codes.PROGRAM_RUNNING,
                f"a controller program was active -- {summary}; clear again once it has stopped")

    def _apply_action(self, action: int, now: float | None = None) -> tuple[bool, int, str]:
        """Dispatch a one-shot ARM_ACTION to the arm. Runs on the servo thread."""
        now = self._clock() if now is None else now
        if action == ARM_ACTION_MOTION_ENABLE:
            self._arm.set_motion_enabled(True)
            return (True, 0, "")
        if action == ARM_ACTION_MOTION_DISABLE:
            self._arm.set_motion_enabled(False)
            return (True, 0, "")
        if action == ARM_ACTION_MANUAL_ON:
            self._arm.set_manual_mode(True)
            return (True, 0, "")
        if action == ARM_ACTION_MANUAL_OFF:
            self._arm.set_manual_mode(False)
            return (True, 0, "")
        if action == ARM_ACTION_HOME:
            self._arm.go_home()
            return (True, 0, "")
        if action == ARM_ACTION_HOME_CANCEL:
            # Cancel an in-flight home move. stop_motion halts it; the post-action
            # bookkeeping below releases the home standdown latch so the loop
            # re-enters mode-7 control on its next reconcile.
            self._arm.stop_motion()
            return (True, 0, "")
        if action == ARM_ACTION_RECOVER:
            blocked = self._program_blocks_clear(now, "recover")
            return blocked if blocked is not None else self._arm.clear_error()
        if action == ARM_ACTION_REARM:
            # Manual re-arm/resume: re-assert control mode + state 0. Used when
            # auto re-arm is OFF and the operator wants to resume after an idle.
            self._arm.rearm()
            return (True, 0, "")
        if action == ARM_ACTION_AUTOREARM_ON:
            self._auto_rearm = True     # no arm SDK call; just a loop-policy flag
            return (True, 0, "")
        if action == ARM_ACTION_AUTOREARM_OFF:
            self._auto_rearm = False
            return (True, 0, "")
        if action == ARM_ACTION_BACKOUT:
            return self._do_backout(now)
        if action == LOCAL_ACTION_SET_HOME:
            # Dashboard "set home here" (server-internal, not a wire action).
            # A home outside the envelope would make every HOME a bad move (the
            # folded-posture incident, 2026-10-12): refuse it with the reason.
            reason = self._posture_violation(self._arm.read_snapshot())
            if reason:
                return (False, error_codes.OUTSIDE_ENVELOPE,
                        f"refused: the arm is outside the envelope ({reason}); move it "
                        "inside first (RETREAT or hand-guide), then set home")
            return self._arm.set_home_here()
        if action == ARM_ACTION_RETREAT:
            return self._do_retreat()
        return (False, -2, f"unknown arm action {action}")

    def _do_retreat(self) -> tuple[bool, int, str]:
        """Operator-triggered RETREAT: a slow planned move to the nearest
        in-envelope posture -- joints clamped into the bridge limits first (the
        folded elbow), else the TCP pushed out of the box edge / reach limit /
        keep-out with a margin. Never automatic; STOP cancels it."""
        self._retreat_moving = False
        if not getattr(self._arm, "supports_retreat", False):
            return (False, error_codes.UNSUPPORTED, "retreat is not supported by this arm backend")
        snap = self._arm.read_snapshot()
        if snap.arm_state in (ArmState.UNKNOWN, ArmState.ERROR, ArmState.PROTECTIVE_STOP):
            return (False, error_codes.NOT_READY,
                    f"cannot retreat while the arm is {snap.arm_state.name}; clear the fault first")
        ws = self._cfg.workspace
        try:
            limits = self._arm.joint_limits_deg()
        except Exception:  # noqa: BLE001
            limits = None
        joints = safety.retreat_joints(snap.joints, limits, ws.retreat_margin_deg)
        pose = None
        if joints is None:
            pose = safety.retreat_pose(snap.pose, ws, ws.retreat_margin_mm)
            if pose is None:
                self._refresh_envelope(snap)
                return (True, 0, "the arm is inside the envelope; nothing to retreat from")
        self._arm.retreat_to(joints, pose)
        self._retreat_moving = True
        what = (f"joints -> {_fmt_joints(joints)}" if joints is not None
                else f"TCP -> {_fmt_pose(pose)}")
        log.warning("arm: RETREAT started: %s", what)
        return (True, 0, f"retreating ({what}); STOP cancels")

    def _do_backout(self, now: float | None = None) -> tuple[bool, int, str]:
        """Operator-triggered undo: clear the fault, then queue the recorded
        trail REVERSED for the loop to replay (the reverse of a just-traversed
        path is the safest known retreat -- standard for collision faults).
        With no fault (a "target held" warning) the clear is a no-op and it
        just backs up. Backends with `supports_joint_replay` replay the
        recorded JOINTS along a time-parameterized path (no IK: it works at
        and through a singularity, where the pose replay could not solve);
        others stream the poses. Clear-only after an e-stop (auto-motion right
        after an e-stop release is never right) or when nothing was recorded.
        The queue is executed by `_run_backout_step`; a fresh clutch edge is
        still required afterwards."""
        blocked = self._program_blocks_clear(self._clock() if now is None else now, "backout")
        if blocked is not None:
            return blocked
        estop = False
        try:
            estop = self._arm.read_snapshot().error_code in ESTOP_ERROR_CODES
        except ArmFault:
            pass                        # unreadable snapshot: treat as non-e-stop
        ok, code, msg = self._arm.clear_error()
        if not ok:
            return (False, code, msg)
        with self._fault_lock:          # cleared: unlatch, like CLEAR_ERROR/RECOVER
            self._faulted = False
            self._fault_code = 0
            self._fault_msg = ""
        if estop:
            return (True, 0, "cleared; no back-out after an e-stop")
        trail = list(reversed(self._trail))
        self._trail.clear()             # the trail is consumed; do not replay twice
        if not trail:
            return (True, 0, "cleared; no recorded motion to back out")
        self._backout_wait_since = None
        path = self._joint_backout_path(trail)
        if path is not None:
            self._backout_joints = True
            self._backout_queue = path
            return (True, 0, f"cleared; backing out {len(trail)} points "
                             f"({len(path) * self._cfg.servo_dt:.1f} s, joint replay)")
        self._backout_joints = False
        self._backout_queue = [pose for _t, pose, _q in trail]
        return (True, 0, f"cleared; backing out {len(trail)} points")

    def _joint_backout_path(self, trail) -> list | None:
        """The reversed trail as one joint point per tick from the arm's
        actual joints (safety.joint_path_profile at the backend's replay
        speed), or None to replay poses instead (backend without joint replay,
        or a trail/snapshot without joints)."""
        if not getattr(self._arm, "supports_joint_replay", False):
            return None
        if any(q is None for _t, _p, q in trail):
            return None
        try:
            start = self._arm.read_snapshot().joints
            v_max, a_max = self._arm.joint_replay_limits_deg()
        except ArmFault:
            return None
        if not start or v_max <= 0 or a_max <= 0:
            return None
        return safety.joint_path_profile([start] + [q for _t, _p, q in trail],
                                         v_max, a_max, self._cfg.servo_dt)

    def tick(self, now: float | None = None) -> None:
        now = self._clock() if now is None else now
        self._jr_plan = None
        self._jr_snap = None
        try:
            self._tick(now)
        finally:
            self._journal_tick(now)
            self._capture_tick(now)

    def _capture_tick(self, now: float) -> None:
        """Hand this tick's command to the capture tap (dataset `command_exec`):
        the plan used, the deadman, the mode and what the backend actually sent.
        O(1); like the journal it must never stop the loop."""
        tap = self._capture
        if tap is None or not tap.listening:
            return
        try:
            tap.tick(int(now * 1e9), self._jr_plan, self.mode,
                     self._session.deadman_held, self._arm.journal_fields())
        except Exception as exc:  # noqa: BLE001 -- capture must never stop the loop
            log.debug("capture: tick record failed: %r", exc)

    def _tick(self, now: float) -> None:
        # Backend per-tick hook (multi-second sequences, slow state polls) --
        # here, so it stays on the servo thread. Contractually non-raising; an
        # ArmFault is still latched rather than allowed to kill the tick.
        try:
            self._arm.service(now)
        except ArmFault as exc:
            self._enter_fault(exc.code, exc.message)
        stops = self._take_stops()
        if stops:
            summary = self._do_stop(now, stops[0][0])
            for _source, pending in stops:
                pending.fulfill(True, summary)
        plan = self._session.plan(now)
        self._jr_plan = plan

        # CLEAR_ERROR runs here so the arm SDK is only ever touched on this thread.
        pending = self._session.take_clear_request()
        if pending is not None:
            try:
                blocked = self._program_blocks_clear(now, "quest clear")
                ok, code, msg = blocked if blocked is not None else self._arm.clear_error()
            except ArmFault as exc:
                ok, code, msg = False, exc.code, exc.message
            pending.fulfill(ControlResult(ok, code, msg))
            if ok:
                # Recovered: unlatch and force a full re-reconcile. Control mode is
                # re-entered only if enable intent still holds, and driving needs a
                # fresh clutch edge -- a clear never resumes motion.
                with self._fault_lock:
                    self._faulted = False
                    self._fault_code = 0
                self._control_on = False
                self._was_driving = False
                self._last_sent_pose = None
                self._need_rearm = True

        # One-shot ARM_ACTION also runs here, on the servo thread.
        action = self._session.take_action_request()
        if action is not None and action.action in _MODE_ACTIONS:
            try:
                ok, code, msg = self._apply_mode(_MODE_ACTIONS[action.action], now)
            except ArmFault as exc:
                ok, code, msg = False, exc.code, exc.message
            action.fulfill(ControlResult(ok, code, msg))
            self._note_mode(now, "dashboard")
            action = None
        # Who asked (for the mode attribution) and which action it really is.
        from_dashboard = action is not None and action.action >= LOCAL_ACTION_SET_HOME
        act = _LOCAL_WIRE.get(action.action, action.action) if action is not None else None
        program_busy = self._programs is not None and self._programs.busy
        # The dashboard's Home / Retreat are fine in Program mode while no program
        # runs (the operator, not a program, moves the arm); the Quest's is not.
        dashboard_home = (from_dashboard and act in (ARM_ACTION_HOME, ARM_ACTION_RETREAT)
                          and not program_busy)
        if (action is not None and self._program_mode
              and act not in _PROGRAM_MODE_ACTIONS and not dashboard_home):
            action.fulfill(ControlResult(
                False, error_codes.PROGRAM_MODE,
                "a program is running -- stop it first" if program_busy and from_dashboard
                else "the station is in Program mode -- switch to Teleop on the dashboard first"))
            action = None
        if action is not None:
            if self._program_mode and self._programs is not None and act in (
                    ARM_ACTION_HOME_CANCEL, ARM_ACTION_MOTION_DISABLE):
                self._programs.stop_now(now, "dashboard" if from_dashboard else "quest")
            try:
                ok, code, msg = self._apply_action(act, now)
            except ArmFault as exc:
                ok, code, msg = False, exc.code, exc.message
            action.fulfill(ControlResult(ok, code, msg))
            # Standdown-latch bookkeeping tracks operator INTENT, not just SDK
            # success. AUTOREARM_ON/OFF are pure loop-policy flags, REARM is an
            # in-place resume and SET_HOME only records a pose (it is allowed
            # in manual/free-drive) -- none re-reconcile control state.
            # Any action except BACKOUT itself cancels an in-flight back-out
            # replay -- the operator has expressed a different intent (and
            # HOME_CANCEL's stop_motion is the documented way to abort one).
            if act != ARM_ACTION_BACKOUT:
                self._backout_queue = []
            if act == ARM_ACTION_MANUAL_ON:
                # Park the loop ONLY if teaching mode was actually entered: a
                # failed enter must not freeze the arm in a standdown with no
                # free-drive to show for it.
                if ok:
                    self._manual_active = True
                    self._home_until_idle = False
                    self._control_on = False
                    self._was_driving = False
                    self._last_sent_pose = None
                    # The arm is about to be hand-moved with recording parked;
                    # the trail no longer describes the path to the current pose.
                    self._trail.clear()
            elif act not in (ARM_ACTION_AUTOREARM_ON,
                                       ARM_ACTION_AUTOREARM_OFF, ARM_ACTION_REARM,
                                       LOCAL_ACTION_SET_HOME):
                # MANUAL_OFF / HOME / HOME_CANCEL / ENABLE / DISABLE / RECOVER all
                # express "leave manual", so release the manual standdown latch
                # EVEN IF THE SDK CALL FAILED. Otherwise a failed MANUAL_OFF (far
                # more likely right after the arm was hand-moved) left the loop
                # parked in manual forever, ignoring the clutch while the client
                # showed manual off. The reconcile below re-asserts mode 7 and the
                # health/fault gate remains the safety net. HOME arms its own
                # async-move standdown only when the move actually started (ok).
                self._manual_active = False
                # RETREAT is a planned move like HOME: same standdown until idle.
                self._home_until_idle = ok and (
                    act == ARM_ACTION_HOME
                    or (act == ARM_ACTION_RETREAT and self._retreat_moving))
                if self._home_until_idle:
                    self._home_action_at = now
                    # The home move runs with recording parked; drop the trail so
                    # a later BACKOUT cannot replay a path that predates it.
                    self._trail.clear()
                self._control_on = False
                self._was_driving = False
                self._last_sent_pose = None
            if ok and act == ARM_ACTION_REARM:
                # Manual resume (auto re-arm off): the controller is re-armed now;
                # clear the hold and settle so the loop drives again on its own.
                self._need_rearm = False
                self._ready_at = now + SETTLE_S
                self._last_sent_pose = None
            if ok and act in (ARM_ACTION_MOTION_ENABLE, ARM_ACTION_RECOVER):
                with self._fault_lock:
                    self._faulted = False
                    self._fault_code = 0
            self._note_mode(now, "dashboard" if from_dashboard else "quest")

        # Program runs: commands, controller state, watchdog. Before the fault
        # gate -- a faulted run must still be seen to end, and Stop must work.
        if self._programs is not None:
            self._programs.service(now, self._program_mode,
                                   arm_moving=self._home_until_idle or bool(self._backout_queue))

        # While faulted, hold: touch the arm only via CLEAR_ERROR / ARM_ACTION
        # (both can revive it). NO auto-clear -- the operator recovers explicitly.
        if self._is_faulted:
            self._was_driving = False
            return

        # Manual/free-drive engaged: stand down completely (operator owns the arm
        # by hand). Resumes only after MANUAL_OFF / home / enable / recover.
        if self._manual_active:
            self._was_driving = False
            return

        # HOME move in flight: stand down until it finishes (a mode switch would
        # terminate it). Released once the grace period passed and the arm is idle.
        if self._home_until_idle:
            self._was_driving = False
            try:
                if (now - self._home_action_at) >= 1.0 \
                        and not self._arm.read_snapshot().moving:
                    self._home_until_idle = False
            except ArmFault as exc:
                self._enter_fault(exc.code, exc.message)
            return

        # Program mode: the controller program owns the arm; stream nothing.
        # (After the HOME stand-down, so a Home started in Program mode is
        # still seen to finish.)
        if self._program_mode:
            self._was_driving = False
            return

        try:
            # Health gate: UNKNOWN (stale/no report), ERROR, or PROTECTIVE_STOP
            # must not be driven. Hold; the operator clears a real fault. (State
            # 4/5/6 without an error map to READY, not ERROR -- see _map_state --
            # so a normal mode-7 idle is re-armed below, not treated as a fault.)
            snap = self._arm.read_snapshot()
            self._jr_snap = snap
            if snap.arm_state in (ArmState.UNKNOWN, ArmState.ERROR,
                                  ArmState.PROTECTIVE_STOP):
                self._was_driving = False
                self._backout_queue = []    # unhealthy mid-replay: abort, hold
                return

            # An envelope hold clears itself once the arm is back inside (after
            # RETREAT, or hand-guided out), even with no Quest session -- so the
            # dashboard's Retreat button and warn 1006 never go stale.
            if self._envelope_block is not None and not self._was_driving:
                self._refresh_envelope(snap)

            # BACKOUT replay (operator-triggered undo): while the reversed trail
            # queue is non-empty, the loop streams it instead of the drive plan.
            # Recording is paused so a second BACKOUT cannot "undo the undo".
            # An operator drive (deadman + fresh heartbeat) aborts the replay --
            # teleop intent always wins.
            if self._backout_queue:
                self._was_driving = False
                if plan.drive or not plan.want_servo:
                    # Operator drove (teleop wins) or the session/enable intent
                    # is gone (disconnect / SET_ENABLED(0)) -> abort the replay.
                    self._backout_queue = []
                else:
                    self._run_backout_step(now, snap)
                    return
            else:
                self._record_trail(now, snap.pose, snap.joints)

            # Envelope gate: never enter control (or start a burst) while the
            # arm's actual posture is outside the bridge limits -- the operator
            # retreats or hand-guides it first. Re-checked here so the hold
            # clears by itself once the arm is back inside.
            if plan.want_servo and (not self._control_on or self._need_rearm):
                if self._refresh_envelope(snap):
                    if self._control_on:
                        self._release_control()
                    self._was_driving = False
                    self.last_target = None
                    return

            # Reconcile control mode to intent (mode 7). No churny disable on brief
            # gaps: disable only on an explicit loss of want_servo. enable_servo
            # already re-asserts state 0, so it starts the settle window directly.
            if plan.want_servo and not self._control_on:
                self._arm.enable_servo()   # set_mode(7) + set_state(0)
                self._control_on = True
                self._need_rearm = False
                self._ready_at = now + SETTLE_S
                self._last_sent_pose = None
            elif not plan.want_servo and self._control_on:
                self._arm.disable_servo()
                self._control_on = False
                self._was_driving = False
                self._last_sent_pose = None

            if not (plan.want_servo and self._control_on):
                self._was_driving = False
                return

            if not plan.drive:
                # Deadman released / watchdog / no fresh target: stream nothing
                # (mode 7 holds its last target). The controller idles to state 5
                # (MODE_CHANGED) while we are silent, so require a re-arm before the
                # next drive burst rather than commanding a not-ready controller.
                # A burst that just ended gets one end_stream: a streaming
                # backend (AUBO trackJoint) must stop smoothly rather than leave
                # its last queued point as an instant velocity step -- the
                # 2026-10-12 20:59 release-time collision stop.
                if self._was_driving:
                    reason = self._burst_end_reason(now)
                    self.journal.event(now, "burst_end", reason=reason)
                    self._arm.end_stream(reason)
                self._was_driving = False
                self._need_rearm = True
                self.last_target = None
                return

            assert plan.target_pose is not None and plan.target_gripper is not None

            # Re-arm before driving if the controller may have gone not-ready
            # (burst start, prior not-ready reply, or a silent idle). Re-assert
            # state 0 and settle BEFORE any set_position -- this is what stops the
            # "not ready to move" (code 1) reply and controller error 35. Auto by
            # default and cooldown-limited so it can never storm; with auto re-arm
            # off the loop holds here until the operator presses RESUME (REARM).
            if self._need_rearm:
                self._was_driving = True
                if self._auto_rearm and (now - self._rearm_at) >= REARM_COOLDOWN_S:
                    self._rearm_at = now
                    self._arm.rearm()          # set_mode(7) if needed + set_state(0)
                    self._ready_at = now + SETTLE_S
                    self._last_sent_pose = None
                    self._need_rearm = False
                return                          # let it settle; command next tick
            if now < self._ready_at:
                self._was_driving = True
                return                          # still settling after the re-arm

            # Ready: stream the target on-change. Mode 7 plans the smooth,
            # speed/accel-limited path and holds when we stop sending. A NONZERO
            # reply means the controller went not-ready under a stale SERVO
            # snapshot -- re-arm and retry next tick instead of latching a fault.
            if self._should_send(plan.target_pose):
                code = self._arm.set_servo_cartesian(plan.target_pose) or 0
                if code == 0:
                    self._last_sent_pose = plan.target_pose
                else:
                    self._need_rearm = True
            self.last_target = plan.target_pose
            self._arm.set_gripper(plan.target_gripper)
            self._was_driving = True
        except ArmFault as exc:
            self._enter_fault(exc.code, exc.message)

    def _check_windup(self, joints) -> None:
        """Warn (once per excursion, with hysteresis) when a +/-360 deg joint winds
        past WINDUP_WARN_DEG -- invisible in the 3D view but a latent error-23. Runs
        on the arm-status thread only; the warning reaches the dashboard log panel
        via the arm_bridge logging bridge."""
        if not joints:
            return
        for i in WRAP_JOINT_INDEXES:
            if i >= len(joints):
                continue
            mag = abs(joints[i])
            if mag >= WINDUP_WARN_DEG and i not in self._windup_warned:
                self._windup_warned.add(i)
                log.warning("arm: J%d wound to %.0f deg (limit +/-360); HOME or "
                            "unwind the wrist before it faults (error 23)",
                            i + 1, joints[i])
            elif mag < WINDUP_CLEAR_DEG:
                self._windup_warned.discard(i)

    def _record_trail(self, now: float, pose: Pose, joints=None) -> None:
        """Append the actual pose (and joints) to the rolling BACKOUT trail
        (deduped by motion, pruned to TRAIL_WINDOW_S). Called only on healthy
        ticks with no replay in flight, so the trail freezes automatically on
        fault / manual / home standdown -- the pre-fault path survives until
        the operator acts. Joint motion counts on its own: near a singularity
        the joints swing while the TCP barely moves."""
        joints = tuple(joints) if joints else None
        if self._trail:
            _t, last, last_q = self._trail[-1]
            moved = (max(abs(pose[i] - last[i]) for i in range(3)) >= TRAIL_MIN_STEP_MM
                     or max(abs(pose[i] - last[i]) for i in range(3, 6)) >= TRAIL_MIN_STEP_DEG
                     or (joints is not None and last_q is not None
                         and max(abs(a - b) for a, b in zip(joints, last_q))
                         >= TRAIL_MIN_STEP_DEG))
            if not moved:
                return
        self._trail.append((now, pose, joints))
        cutoff = now - TRAIL_WINDOW_S
        while self._trail and self._trail[0][0] < cutoff:
            self._trail.popleft()

    def _run_backout_step(self, now: float, snap: ArmSnapshot) -> None:
        """Advance the back-out replay by (at most) one waypoint per tick --
        recorded at loop rate, so the replay runs at roughly the original speed,
        capped by the controller's own mode-7 speed limit. Enters/settles mode-7
        control exactly like a drive burst; a rejected send aborts the replay
        (hold; the operator can re-press BACK OUT or drive manually)."""
        if not self._control_on:
            self._arm.enable_servo()
            self._control_on = True
            self._need_rearm = False
            self._ready_at = now + SETTLE_S
            self._last_sent_pose = None
            return
        if now < self._ready_at:
            return
        if self._backout_joints:
            self._run_joint_backout_step(now)
            return
        # Skip waypoints outside the server-owned box (an actual pose can sit
        # marginally outside it, e.g. sag below z_min) and sub-threshold steps.
        while self._backout_queue:
            pose = self._backout_queue[0]
            if safety.validate_target(pose, snap.gripper, self._cfg.workspace) is not None \
                    or not self._should_send(pose):
                self._backout_queue.pop(0)
                continue
            code = self._arm.set_servo_cartesian(pose) or 0
            if code == 0:
                self._last_sent_pose = pose
                self._backout_queue.pop(0)
            else:
                # Controller refused mid-replay: abort and hold rather than
                # retry-loop into another fault.
                log.warning("arm: back-out aborted (set_position code %s, "
                            "%d points left)", code, len(self._backout_queue))
                self._backout_queue = []
                self._need_rearm = True
            break
        if not self._backout_queue:
            # Replay finished (or drained by skips): require a re-arm before the
            # next drive burst, same as after any silence.
            self._need_rearm = True

    def _run_joint_backout_step(self, now: float) -> None:
        """One point of the joint replay per tick (the path is already
        time-parameterized). A point the backend cannot send yet is retried
        (bounded by BACKOUT_WAIT_S); a refused one aborts the replay -- the
        arm holds where it is and the operator re-presses or drives."""
        code = self._arm.set_servo_joints(self._backout_queue[0]) or 0
        if code == 0:
            self._backout_queue.pop(0)
            self._backout_wait_since = None
        elif code == 1:
            if self._backout_wait_since is None:
                self._backout_wait_since = now
            elif now - self._backout_wait_since > BACKOUT_WAIT_S:
                log.warning("arm: back-out aborted (the arm stayed not ready for %.0f s, "
                            "%d points left)", BACKOUT_WAIT_S, len(self._backout_queue))
                self._backout_queue = []
        else:
            log.warning("arm: back-out aborted (joint point refused, code %s, "
                        "%d points left)", code, len(self._backout_queue))
            self._backout_queue = []
            self._arm.end_stream("back-out aborted")   # stop smoothly mid-path
        if not self._backout_queue:
            self._backout_wait_since = None
            self._need_rearm = True

    def _should_send(self, target: Pose) -> bool:
        """On-change gate: send set_position only when the target moved past the
        MIN_SEND thresholds since the last one (mode 7 re-plans per call)."""
        if self._stream_every_tick:
            return True
        prev = self._last_sent_pose
        if prev is None:
            return True
        if max(abs(target[i] - prev[i]) for i in range(3)) > MIN_SEND_MM:
            return True
        if max(abs(target[i] - prev[i]) for i in range(3, 6)) > MIN_SEND_DEG:
            return True
        return False

    def status(self) -> Status:
        """Build a STATUS snapshot from the arm's actual readback (rev 3 sec 5).
        A latched actuator fault overrides the reported state to ERROR so the Quest
        always sees the fault even if the readback path looks otherwise healthy."""
        snap: ArmSnapshot = self._arm.read_snapshot()
        # Cached for the dashboard StateHub (joints etc. -- fields Status does
        # not carry). Written and read on the same arm-status thread; the ref
        # swap is GIL-atomic for anyone else peeking.
        self.last_snapshot = snap
        tap = self._capture
        if tap is not None and self._capture_snapshots and tap.listening:
            tap.snapshot(snap.ts_mono_ns or time.monotonic_ns(), snap)
        self._check_windup(snap.joints)
        x, y, z, roll, pitch, yaw = snap.pose
        arm_state = snap.arm_state
        error_code = snap.error_code
        error_is_api = False
        with self._fault_lock:
            if self._faulted:
                arm_state = ArmState.ERROR
                if error_code == 0:
                    # The robot report shows no controller error, so the number
                    # we surface is the latched ArmFault's code. For the xArm
                    # that is an SDK API return code -- a DIFFERENT namespace
                    # (API 1 = "errors not cleared" vs controller 1 = e-stop),
                    # flagged so the client labels it correctly
                    # (STATUS_FLAG_ERRCODE_IS_API). Newer backends raise teleop
                    # codes (error_codes.py, 2000+), which need no flag.
                    error_code = self._fault_code
                    error_is_api = not error_codes.is_teleop_code(error_code)
        # Surface the documented HOMING state (armState 5) while a home move OR a
        # BACKOUT replay is in flight so the Quest keeps the clutch out (SERVO
        # gate) and shows its CANCEL affordance -- HOME_CANCEL's stop_motion
        # aborts either. A real fault still wins; both flags are servo-thread
        # state read GIL-atomically here.
        if arm_state not in (ArmState.ERROR, ArmState.PROTECTIVE_STOP) \
                and (self._home_until_idle or self._backout_queue):
            arm_state = ArmState.HOMING
        # Program mode (no wire change): a running program is HOMING to the
        # Quest -- clutch kept out, Cancel shown (HOME_CANCEL stops it); idle
        # Program mode is DISABLED (the Quest cannot drive).
        if self._program_mode and arm_state not in (
                ArmState.ERROR, ArmState.PROTECTIVE_STOP, ArmState.UNKNOWN):
            busy = (self._programs is not None and self._programs.busy) \
                or self._home_until_idle or bool(self._backout_queue)
            arm_state = ArmState.HOMING if busy else ArmState.DISABLED
        # The motion permit (§5.6) IS the server-monotonic issue time in ms: the
        # Quest echoes it, and the session ages it against the same clock with no
        # sync needed. u32 wrap (~49 days) is well beyond any session.
        permit = int(self._clock() * 1000.0) & 0xFFFFFFFF
        return Status(
            arm_state=arm_state, moving=snap.moving,
            servo_active=snap.servo_active, protective_stop=snap.protective_stop,
            x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, gripper=snap.gripper,
            error_code=error_code,
            # An envelope hold is a teleop warning (1006) unless the backend
            # already reports one (its own holds come first).
            warn_code=snap.warn_code or (
                error_codes.WARN_OUTSIDE_ENVELOPE if self._envelope_block else 0),
            motion_permit_id=permit,
            # Surface the session's comm/quality hold latch (watchdog gap, stale
            # permit) so the client can tell the operator to release + re-engage
            # -- without this bit a latched hold looks like a healthy SERVO state
            # while the clutch is silently ignored.
            hold_latched=self._session.hold_latched,
            error_is_api=error_is_api,
        )

    # ---- production runner -----------------------------------------------
    def run(self) -> None:
        """Blocking paced loop; call in a dedicated thread."""
        dt = self._cfg.servo_dt
        next_t = self._clock()
        while not self._stop.is_set():
            try:
                self.tick()
            except Exception as exc:  # last-resort backstop: tick catches ArmFault
                # itself; anything else must still not kill the loop, or the
                # watchdog/hold that keeps the arm safe would die with it.
                self._enter_fault(getattr(exc, "code", 0), repr(exc))
            next_t += dt
            sleep = next_t - self._clock()
            if sleep > 0:
                self._stop.wait(sleep)
            else:
                next_t = self._clock()  # fell behind; resync rather than spin

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self.run, name="arm-servo", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        # Leave the arm safe.
        if self._control_on:
            try:
                self._arm.disable_servo()
            except ArmFault:
                pass
            self._control_on = False
