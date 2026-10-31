"""Arm session: what the Quest has asked for, validated (rev 3 sections 4, 5, 8).

The Session holds *intent* only -- who is connected, whether they enabled servo,
and the latest accepted target. It never touches the arm SDK. The servo loop
reads a `DrivePlan` from `plan()` each tick and is the sole actuator, so arm
access stays single-threaded and transport callbacks can never block the loop
(rev 3 section 10: "never let telemetry, logging, or visualization block the
servo loop").

Thread-safety: control-channel callbacks (armctl) and heartbeat callbacks (arm)
run on the transport thread; the servo loop runs on its own thread. All shared
state is guarded by one short-held lock.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from . import safety
from .config import ArmBridgeConfig
from .link_stats import LinkMonitor
from .protocol import Heartbeat

Pose = tuple[float, float, float, float, float, float]

_SEQ_MOD = 1 << 32
_SEQ_HALF = 1 << 31

# Server-internal actions (dashboard-originated; never on the wire). Numbered
# from 100 so they cannot collide with appended ARM_ACTION_* wire codes.
LOCAL_ACTION_SET_HOME = 100
# Station control mode (servo.py): who may command the arm.
LOCAL_ACTION_MODE_TELEOP = 101
LOCAL_ACTION_MODE_FREE_DRIVE = 102
LOCAL_ACTION_MODE_PROGRAM = 103
# Dashboard "Go home": the same controlled HOME move as the Quest's
# ARM_ACTION_HOME (servo.py runs it through the identical path).
LOCAL_ACTION_HOME = 104
# Dashboard "Clear fault": the Quest's ARM_ACTION_RECOVER (clear / unlock a
# protective stop -- an operator action, never automatic).
LOCAL_ACTION_RECOVER = 105
# Dashboard "Retreat": the Quest's ARM_ACTION_RETREAT -- a slow planned move
# back inside the envelope when the arm was hand-guided (or homed) outside it.
# Operator-triggered, never automatic; the servo loop computes the nearest
# in-envelope target.
LOCAL_ACTION_RETREAT = 106
# Dashboard Arm menu "Power on" / "Power off": the Quest's ARM_ACTION_MOTION_
# ENABLE / _DISABLE (on the AUBO a real power cycle: poweron -> brake release
# / stop + poweroff). Operator actions, never automatic.
LOCAL_ACTION_POWER_ON = 107
LOCAL_ACTION_POWER_OFF = 108


@dataclass(frozen=True)
class ControlResult:
    """Outcome of a control request, mapped to a REPLY by the transport."""
    ok: bool
    code: int
    msg: str = ""


@dataclass(frozen=True)
class DrivePlan:
    """What the servo loop should do this tick."""
    want_servo: bool                      # reconcile arm servo mode to this
    drive: bool                           # actively command motion this tick
    target_pose: Pose | None
    target_gripper: float | None


class PendingClear:
    """A CLEAR_ERROR request handed to the servo loop; the transport waits on it
    for the real outcome (the servo thread is the only caller of the arm SDK)."""

    def __init__(self) -> None:
        self._event = threading.Event()
        self.result: ControlResult | None = None

    def fulfill(self, result: ControlResult) -> None:
        self.result = result
        self._event.set()

    def wait(self, timeout: float) -> ControlResult:
        if self._event.wait(timeout) and self.result is not None:
            return self.result
        return ControlResult(False, -1, "clear timed out")


class PendingAction(PendingClear):
    """A one-shot ARM_ACTION (enable/disable/manual/home/recover) handed to the
    servo loop; same wait/fulfill contract as PendingClear, plus the action code
    the loop dispatches on."""

    def __init__(self, action: int) -> None:
        super().__init__()
        self.action = action


class Session:
    def __init__(self, config: ArmBridgeConfig, clock=time.monotonic) -> None:
        self._cfg = config
        self._clock = clock
        self._lock = threading.Lock()
        self._established = False
        self._session_id: int | None = None
        self._peer: str | None = None
        self._enabled_desired = False
        self._last_seq: int | None = None
        self._last_hb_ts = 0.0
        self._target: tuple[Pose, float, bool] | None = None  # pose, gripper, deadman
        # Latched communication/quality hold (safety review; TURN Phase 1): once a
        # watchdog/freshness hold occurs while the clutch is held, motion must not
        # resume on a merely-fresh heartbeat -- the operator has to release the
        # deadman and re-engage. Cleared only by an accepted deadman-release.
        self._hold_latched = False
        # Newest motionPermitId echoed by the Quest (§5.6). 0 = none seen yet.
        self._motion_permit = 0
        self._pending_clear: PendingClear | None = None
        self._pending_action: PendingAction | None = None
        # telemetry / test counters
        self.accepted_count = 0
        self.rejected_count = 0
        # Phase 1 observability (additive; no effect on drive/hold decisions).
        # Gaps are measured between consecutively accepted heartbeats only.
        self.hb_gap_max_ms = 0.0        # largest inter-arrival gap seen this session
        self.hb_watchdog_gaps = 0       # accepted gaps that exceeded the 150 ms watchdog
        self.seq_skips = 0              # total sequence numbers skipped (loss/reorder)
        self.permit_age_max_ms = 0.0    # largest motion-permit age seen (data to tighten §5.6)
        self.permit_holds = 0           # times an expired permit forced a hold
        # Dashboard link quality (display only). Fed after the session lock is
        # released, so its own lock never nests inside ours.
        self.link = LinkMonitor(config.link_quality, clock=clock)

    # ---- control channel (armctl) ----------------------------------------
    def on_hello(self, session_id: int, peer: str) -> ControlResult:
        """Establish (or replace) the active session. HELLO is always first
        (rev 3 section 5); a new HELLO resets sequence tracking (section 10)."""
        with self._lock:
            self._established = True
            self._session_id = session_id & 0xFFFFFFFF
            self._peer = peer
            self._enabled_desired = False
            self._last_seq = None
            self._target = None
            self._hold_latched = False
            self._motion_permit = 0
            self._last_hb_ts = self._clock()
        self.link.on_session_start()
        # HELLO success returns the negotiated protocol version as its code:
        # the configured one (3 by default; 4 asks for rev 0x04 heartbeats).
        return ControlResult(True, self._cfg.protocol_version, "")

    def on_set_enabled(self, enabled: bool) -> ControlResult:
        """Idempotent (rev 3 section 5). Records intent; the servo loop reconciles
        the actual xArm servo mode. Rejected before HELLO."""
        with self._lock:
            if not self._established:
                return ControlResult(False, 1, "SET_ENABLED before HELLO")
            self._enabled_desired = enabled
        return ControlResult(True, 0, "")

    def request_clear_error(self, now: float | None = None) -> PendingClear | ControlResult:
        """Queue a CLEAR_ERROR for the servo loop. Returns a PendingClear the
        transport waits on, or a ControlResult if it can be rejected now.

        Rejected while actively driving (deadman held + fresh heartbeat): clearing
        a fault must not happen mid-motion, and it must never imply re-enable. The
        operator releases the clutch, then clears; a fresh clutch edge is required
        before motion resumes (safety review)."""
        now = self._clock() if now is None else now
        with self._lock:
            if not self._established:
                return ControlResult(False, 1, "CLEAR_ERROR before HELLO")
            if self._is_driving_locked(now):
                return ControlResult(False, 1,
                                     "CLEAR_ERROR rejected while driving; release clutch first")
            pending = PendingClear()
            self._pending_clear = pending
            return pending

    def request_arm_action(self, action: int,
                           now: float | None = None) -> "PendingAction | ControlResult":
        """Queue a one-shot ARM_ACTION for the servo loop (the only arm-SDK
        caller). Returns a PendingAction the transport waits on, or a
        ControlResult if it can be rejected now. Rejected before HELLO and while
        actively driving (release the clutch first) -- same safety rule as
        CLEAR_ERROR: arm mode/power must not change mid-motion."""
        now = self._clock() if now is None else now
        with self._lock:
            if not self._established:
                return ControlResult(False, 1, "ARM_ACTION before HELLO")
            if action >= LOCAL_ACTION_SET_HOME:
                # Server-internal (dashboard, PIN-gated) codes never come off
                # the wire: the u8 would otherwise reach them.
                return ControlResult(False, 1, f"unknown arm action {action}")
            if self._is_driving_locked(now):
                return ControlResult(False, 1,
                                     "ARM_ACTION rejected while driving; release clutch first")
            pending = PendingAction(action)
            self._pending_action = pending
            return pending

    def request_local_action(self, action: int,
                             now: float | None = None) -> "PendingAction | ControlResult":
        """Queue a server-internal action (LOCAL_ACTION_*) from the dashboard.
        Unlike request_arm_action it needs no Quest session (the station may be
        operated from the dashboard alone), but the same rule holds: rejected
        while a session is actively driving."""
        now = self._clock() if now is None else now
        with self._lock:
            if self._is_driving_locked(now):
                return ControlResult(False, 1,
                                     "action rejected while driving (release the clutch)")
            if self._pending_action is not None:
                return ControlResult(False, 1, "another arm action is pending")
            pending = PendingAction(action)
            self._pending_action = pending
            return pending

    def take_action_request(self) -> "PendingAction | None":
        with self._lock:
            pending, self._pending_action = self._pending_action, None
            return pending

    def _is_driving_locked(self, now: float) -> bool:
        """Actively commanding motion right now: enabled, deadman held, fresh
        heartbeat. Caller must hold self._lock."""
        if not (self._established and self._enabled_desired) or self._target is None:
            return False
        _pose, _gripper, deadman = self._target
        fresh = (now - self._last_hb_ts) <= self._cfg.watchdog_s
        return bool(deadman and fresh)

    def on_disconnect(self) -> None:
        """TCP/armctl gone: invalidate the session and hold (rev 3 section 8).
        UDP/arm packets are rejected until a new HELLO."""
        with self._lock:
            self._established = False
            self._session_id = None
            self._peer = None
            self._enabled_desired = False
            self._last_seq = None
            self._target = None
            self._hold_latched = False
            self._motion_permit = 0
        self.link.on_session_end()

    # ---- command channel (arm heartbeat) ---------------------------------
    def on_heartbeat(self, hb: Heartbeat, peer: str, now: float | None = None) -> tuple[bool, str]:
        """Validate and store one heartbeat. Returns (accepted, reason)."""
        now = self._clock() if now is None else now
        with self._lock:
            if not self._established:
                self.rejected_count += 1
                return (False, "no active session")
            if hb.session_id != self._session_id:
                self.rejected_count += 1
                return (False, "sessionId mismatch")
            if self._peer is not None and peer != self._peer:
                self.rejected_count += 1
                return (False, "source mismatch")
            if self._last_seq is not None and self._is_stale(hb.seq):
                self.rejected_count += 1
                return (False, "stale seq")
            # Phase 1 telemetry: measure the gap from the previous live
            # heartbeat (skipped on the first packet, which has no predecessor).
            gap_ms: float | None = None
            skipped = 0
            if self._last_seq is not None:
                gap_ms = (now - self._last_hb_ts) * 1000.0
                if gap_ms > self.hb_gap_max_ms:
                    self.hb_gap_max_ms = gap_ms
                if gap_ms > self._cfg.watchdog_ms:
                    self.hb_watchdog_gaps += 1
                advance = (hb.seq - self._last_seq) % _SEQ_MOD
                if advance > 1:
                    skipped = advance - 1
                    self.seq_skips += skipped
            self._last_seq = hb.seq
            # Validity is NOT liveness. A well-formed, in-session heartbeat whose
            # TARGET fails the workspace box is still proof the client is alive
            # and still carries its deadman/permit. Store the pose only when valid
            # (the arm holds at the last valid target -- a soft stop at the box
            # edge); refresh liveness, permit and the deadman regardless. Before
            # this split an out-of-box target (e.g. z < 0 from a hand dip at 1:1
            # scale with a low anchor) starved the watchdog, latched the hold,
            # and -- because the RELEASE heartbeat carried the same out-of-box
            # target -- the release was rejected too, so the latch stuck until
            # the operator re-clutched from an in-box pose (2026-09-11).
            rej = safety.validate_target(hb.pose, hb.gripper, self._cfg.workspace)
            if rej is None:
                self._target = (hb.pose, hb.gripper, hb.deadman)
                self.accepted_count += 1
            else:
                self.rejected_count += 1
                if self._target is not None:
                    pose, gripper, _deadman = self._target
                    self._target = (pose, gripper, hb.deadman)
                # No valid target yet: nothing to hold at, stays None (no drive).
            self._motion_permit = hb.motion_permit_id
            self._last_hb_ts = now
            # A real deadman release clears any latched comm/quality hold; the
            # operator must then re-engage to drive again (safety review).
            if not hb.deadman:
                self._hold_latched = False
        self.link.on_heartbeat(now, gap_ms, skipped, hb.motion_permit_id,
                               target_ok=rej is None)
        return (rej is None, rej.reason if rej is not None else "")

    def _is_stale(self, seq: int) -> bool:
        # Stale if (seq - lastSeq) mod 2^32 is 0 or >= 2^31 (rev 3 section 4).
        diff = (seq - self._last_seq) % _SEQ_MOD
        return diff == 0 or diff >= _SEQ_HALF

    def latch_hold(self) -> None:
        """Force a re-engage requirement (called by the servo loop after an
        auto-recovered fault): motion stays held until the operator releases the
        deadman and clutches again, so the arm never resumes driving straight
        back into the fault under a continuously-held clutch. Cleared only by an
        accepted deadman release (on_heartbeat) -- same latch as a comm/quality
        hold."""
        with self._lock:
            self._hold_latched = True

    # ---- servo loop view --------------------------------------------------
    def plan(self, now: float | None = None) -> DrivePlan:
        now = self._clock() if now is None else now
        latched: str | None = None
        with self._lock:
            want_servo = self._established and self._enabled_desired
            drive = False
            pose = gripper = None
            if want_servo and self._target is not None:
                _pose, _gripper, deadman = self._target
                fresh = (now - self._last_hb_ts) <= self._cfg.watchdog_s
                permit_ok = self._permit_fresh_locked(now)
                # A watchdog/freshness/permit hold while the clutch is still held
                # latches: the operator lost a fresh path mid-drive, so motion must
                # not silently resume when packets return (safety review). Cleared
                # only by an accepted deadman release in on_heartbeat.
                if deadman and (not fresh or not permit_ok):
                    if not self._hold_latched:
                        latched = "watchdog" if not fresh else "permit"
                    self._hold_latched = True
                    if not permit_ok:
                        self.permit_holds += 1
                # Drive only with the deadman held, a fresh heartbeat, a fresh
                # motion permit, and no latched hold (rev 3 section 4 + §5.6).
                if deadman and fresh and permit_ok and not self._hold_latched:
                    drive = True
                    pose, gripper = _pose, _gripper
        if latched is not None:
            self.link.on_hold_latched(latched)
        return DrivePlan(want_servo, drive, pose, gripper)

    def _permit_fresh_locked(self, now: float) -> bool:
        """Whether the last echoed motion permit is fresh by the server clock
        (§5.6). Caller holds self._lock. A 0 permit (Quest has not received a
        STATUS yet) is a loose grace — it does not hold; only a stale non-zero
        permit does. The permit id IS the server-monotonic issue time in ms, so
        no per-id table is needed and there is no clock-sync dependency."""
        if self._motion_permit == 0:
            return True
        now_ms = int(now * 1000.0) & 0xFFFFFFFF
        age_ms = (now_ms - self._motion_permit) % (1 << 32)
        if age_ms > self.permit_age_max_ms:
            self.permit_age_max_ms = float(age_ms)
        return age_ms <= self._cfg.motion_permit_max_age_ms

    def take_clear_request(self) -> PendingClear | None:
        with self._lock:
            pending, self._pending_clear = self._pending_clear, None
            return pending

    # ---- introspection (telemetry/tests) ---------------------------------
    @property
    def established(self) -> bool:
        with self._lock:
            return self._established

    @property
    def enabled_desired(self) -> bool:
        with self._lock:
            return self._enabled_desired

    @property
    def hold_latched(self) -> bool:
        """A comm/quality hold is latched: drive stays held until the operator
        releases the deadman and re-engages (safety review; TURN Phase 1)."""
        with self._lock:
            return self._hold_latched

    @property
    def deadman_held(self) -> bool:
        """Latest accepted heartbeat's deadman bit (False with no target yet).
        Observability only (dashboard) -- drive gating goes through plan()."""
        with self._lock:
            if self._target is None:
                return False
            return bool(self._target[2])

    def hb_age_ms(self, now: float | None = None) -> float | None:
        """Milliseconds since the last accepted heartbeat; None before the
        first one. Observability only (dashboard state snapshot)."""
        now = self._clock() if now is None else now
        with self._lock:
            if not self._established or self._last_hb_ts <= 0.0:
                return None
            return (now - self._last_hb_ts) * 1000.0

    def watchdog_expired(self, now: float | None = None) -> bool:
        now = self._clock() if now is None else now
        with self._lock:
            if not self._established or self._target is None:
                return False
            return (now - self._last_hb_ts) > self._cfg.watchdog_s
