"""Bridge side of the data-capture hand-off (CAPTURE-SYNC-PLAN.md S3).

The bridge stays ROS-free: it hands timestamped arm records to the station's
capture node (teleop_capture), which publishes them under /capture/arm/** while
a recording runs. Design rules R1/R3/R4 of the plan:

- **Hand-off only.** The transport thread (heartbeats), the servo thread
  (ticks), the status thread and the AUBO poller only append a raw tuple to a
  bounded deque -- O(1), GIL-atomic, never raises. Packing, unit conversion and
  the socket write happen on this module's own drain thread.
- **Drop, count, never block.** The deque is bounded (oldest dropped); a send
  that would block, or a missing listener, drops the record. Every record
  carries a tap sequence number, so the capture node sees every gap.
- **Inert while nothing listens.** The drain thread probes the listener once a
  second; until it is there, appends are skipped entirely (one attribute read).

Transport: AF_UNIX SOCK_DGRAM in the Linux abstract namespace
(`\\0teleop-capture-arm` by default). Every station container shares the host
network namespace, so the capture node in another container reaches it with no
shared mount.

Wire format (little-endian, one record per datagram; the capture node mirrors
it -- change both together):

    header  <BBHIQ   version(=1), kind, reserved(0), tap_seq u32, t_mono_ns u64
    RX      <BBBBIII7fQQQ  heartbeat version, flags, timeFlags, reason,
                            sessionId, seq, motionPermitId,
                            x y z (m), roll pitch yaw (rad), gripper (m),
                            poseTimeNs, shownCaptureNs[0], shownCaptureNs[1]
            t_mono_ns = bridge receipt time of the heartbeat
    EXEC    <BBBB7f6f7fff  flags, mode, n_qc, reserved,
                            target x y z roll pitch yaw gripper (m/rad; NaN = none),
                            commanded pose (m/rad; NaN = none),
                            commanded joints[7] (rad; first n_qc valid),
                            linear speed (m/s), angular speed (rad/s)
            t_mono_ns = servo tick time
    STATE   <BBBB7f7f6ff16s16s  n_q, flags, arm_state, reserved,
                            joints[7] (rad), joint speeds[7] (rad/s),
                            tcp x y z roll pitch yaw (m/rad), gripper (m),
                            robot mode, safety mode (ASCII, NUL-padded)
            t_mono_ns = acquisition time of the joint read
    CLOCK   <qI     offset_ns (station - Quest), rtt_us (exchange used)
            t_mono_ns = receipt time of the CLIENT_STATS that carried it

All times are station CLOCK_MONOTONIC nanoseconds except poseTimeNs (the
Quest's clock; the capture node maps it with the latest CLOCK record).
"""

from __future__ import annotations

import itertools
import logging
import math
import socket
import struct
import threading
import time
from collections import deque

log = logging.getLogger("arm_bridge.system.capture")

RECORD_VERSION = 1
KIND_RX = 1
KIND_EXEC = 2
KIND_STATE = 3
KIND_CLOCK = 4

HEADER_FMT = "<BBHIQ"
RX_FMT = "<BBBBIII7fQQQ"
EXEC_FMT = "<BBBB7f6f7fff"
STATE_FMT = "<BBBB7f7f6ff16s16s"
CLOCK_FMT = "<qI"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
MAX_JOINTS = 7

# RX reason codes (Session.on_heartbeat's reject reasons).
REASON_ACCEPTED = 0
REASON_NO_SESSION = 1
REASON_SESSION_MISMATCH = 2
REASON_SOURCE_MISMATCH = 3
REASON_STALE_SEQ = 4
REASON_TARGET_REJECTED = 5
_REASONS = {
    "": REASON_ACCEPTED,
    "no active session": REASON_NO_SESSION,
    "sessionId mismatch": REASON_SESSION_MISMATCH,
    "source mismatch": REASON_SOURCE_MISMATCH,
    "stale seq": REASON_STALE_SEQ,
}

# EXEC flags
EXEC_DRIVE = 0x01
EXEC_DEADMAN = 0x02
EXEC_HELD = 0x04
EXEC_LATCHED = 0x08
EXEC_STOPPING = 0x10
EXEC_HAS_TARGET = 0x20
EXEC_HAS_CMD = 0x40
MODE_CODES = {"teleop": 0, "free_drive": 1, "program": 2}
MODE_UNKNOWN = 255

# STATE flags
STATE_HAS_SPEEDS = 0x01
STATE_HAS_TCP = 0x02
ARM_STATE_UNKNOWN = 255

DEFAULT_SOCKET_NAME = "teleop-capture-arm"
PROBE_PERIOD_S = 1.0
_NAN = float("nan")
_MM = 0.001
_DEG = math.pi / 180.0


def reason_code(reason: str) -> int:
    """Map a Session.on_heartbeat reason to its RX code. Any other text is a
    safety.validate_target rejection (out of box, non-finite, ...)."""
    return _REASONS.get(reason, REASON_TARGET_REJECTED)


def _pose_si(pose) -> tuple:
    """mm/deg pose -> m/rad (NaN x6 for None)."""
    if pose is None:
        return (_NAN,) * 6
    x, y, z, roll, pitch, yaw = pose
    return (x * _MM, y * _MM, z * _MM, roll * _DEG, pitch * _DEG, yaw * _DEG)


def _pad(values, scale: float = 1.0) -> tuple[int, tuple]:
    """(count, exactly MAX_JOINTS floats scaled, NaN-padded)."""
    if not values:
        return 0, (_NAN,) * MAX_JOINTS
    vals = [float(v) * scale for v in list(values)[:MAX_JOINTS]]
    return len(vals), tuple(vals + [_NAN] * (MAX_JOINTS - len(vals)))


def _ascii16(text) -> bytes:
    return str(text or "").encode("ascii", "replace")[:16]


def _num(value) -> float:
    return _NAN if value is None else float(value)


class ArmCaptureTap:
    """Non-blocking hand-off of arm records to the capture node. Construct,
    `start()`, hand to the producers; every producer method is safe to call
    from any thread and never raises."""

    def __init__(self, socket_name: str = DEFAULT_SOCKET_NAME,
                 maxlen: int = 4096) -> None:
        self._address = "\0" + socket_name
        self._queue: deque = deque(maxlen=maxlen)
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._seq = itertools.count(1)
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None
        # Read by producers without a lock (a bool swap is GIL-atomic).
        self.listening = False
        # counters (diagnostics; the capture node also sees tap_seq gaps)
        self.sent = 0
        self.dropped = 0

    # ---- lifecycle --------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="capture-tap", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._wake.set()
        thread = self._thread
        self._thread = None
        if thread is not None:
            thread.join(2.0)
        self._close()

    # ---- producers (any thread; O(1), never raise) ------------------------
    def heartbeat(self, rx_ns: int, hb, reason: str) -> None:
        """Transport thread, after Session.on_heartbeat decided."""
        if self.listening:
            self._queue.append((KIND_RX, rx_ns, hb, reason))
            self._wake.set()

    def tick(self, tick_ns: int, plan, mode: str, deadman: bool, fields: dict) -> None:
        """Servo thread, once per tick (the command actually sent)."""
        if self.listening:
            self._queue.append((KIND_EXEC, tick_ns, plan, mode, deadman, fields))
            self._wake.set()

    def state(self, acq_ns: int, joints_rad, speeds_rad_s, tcp_si,
              robot_mode: str = "", safety_mode: str = "",
              arm_state: int = ARM_STATE_UNKNOWN, gripper_mm: float | None = None) -> None:
        """A backend's measured state, stamped at acquisition (AUBO poller, or
        the status thread for backends without their own state stream)."""
        if self.listening:
            self._queue.append((KIND_STATE, acq_ns, joints_rad, speeds_rad_s, tcp_si,
                                robot_mode, safety_mode, arm_state, gripper_mm))
            self._wake.set()

    def snapshot(self, acq_ns: int, snap) -> None:
        """An ArmSnapshot (mm/deg pose, degree joints) as a STATE record; the
        unit conversion happens on the drain thread."""
        if self.listening:
            self._queue.append((KIND_STATE, acq_ns, snap))
            self._wake.set()

    def clock(self, rx_ns: int, offset_ns: int, rtt_us: int) -> None:
        """The Quest's PING/PONG clock offset (from CLIENT_STATS)."""
        if self.listening:
            self._queue.append((KIND_CLOCK, rx_ns, offset_ns, rtt_us))
            self._wake.set()

    # ---- drain thread -----------------------------------------------------
    def _run(self) -> None:
        next_probe = 0.0
        while not self._stop.is_set():
            if not self.listening:
                now = time.monotonic()
                if now >= next_probe:
                    next_probe = now + PROBE_PERIOD_S
                    self._probe()
                if not self.listening:
                    self._stop.wait(0.1)
                    continue
            self._wake.wait(0.5)
            self._wake.clear()
            self._drain()

    def _probe(self) -> None:
        """Connect to the listener; success switches the producers on."""
        self._close()
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            sock.setblocking(False)
            sock.connect(self._address)
        except OSError:
            return
        self._sock = sock
        self.listening = True
        log.info("capture: arm records -> %s", self._address[1:])

    def _drain(self) -> None:
        while self._queue:
            try:
                item = self._queue.popleft()
            except IndexError:
                return
            try:
                data = pack_record(next(self._seq), item)
            except Exception as exc:  # noqa: BLE001 -- a bad record must not stop the tap
                self.dropped += 1
                log.debug("capture: record pack failed: %r", exc)
                continue
            sock = self._sock
            if sock is None:
                return
            try:
                sock.send(data)
                self.sent += 1
            except BlockingIOError:
                self.dropped += 1             # node is behind: drop, never block
            except OSError:
                # Listener gone (recording ended): switch the producers off and
                # discard what is queued -- nothing records it.
                self.listening = False
                self.dropped += len(self._queue) + 1
                self._queue.clear()
                self._close()
                log.info("capture: listener gone; arm records paused")
                return

    def _close(self) -> None:
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass


# ---- packing / unpacking (the wire format above) ---------------------------
def pack_record(tap_seq: int, item: tuple) -> bytes:
    kind = item[0]
    header = struct.pack(HEADER_FMT, RECORD_VERSION, kind, 0,
                         tap_seq & 0xFFFFFFFF, item[1] & 0xFFFFFFFFFFFFFFFF)
    if kind == KIND_RX:
        _, _, hb, reason = item
        x, y, z, roll, pitch, yaw = _pose_si(hb.pose)
        shown = tuple(getattr(hb, "shown_capture_ns", (0, 0)))
        body = struct.pack(
            RX_FMT, getattr(hb, "version", 3) & 0xFF, hb.flags & 0xFF,
            getattr(hb, "time_flags", 0) & 0xFF, reason_code(reason),
            hb.session_id & 0xFFFFFFFF, hb.seq & 0xFFFFFFFF,
            hb.motion_permit_id & 0xFFFFFFFF,
            x, y, z, roll, pitch, yaw, hb.gripper * _MM,
            getattr(hb, "pose_time_ns", 0), shown[0], shown[1])
    elif kind == KIND_EXEC:
        _, _, plan, mode, deadman, fields = item
        flags = EXEC_DEADMAN if deadman else 0
        target = (_NAN,) * 7
        if plan is not None:
            if plan.drive:
                flags |= EXEC_DRIVE
            if plan.target_pose is not None:
                flags |= EXEC_HAS_TARGET
                gripper = plan.target_gripper
                target = _pose_si(plan.target_pose) + (_num(gripper) * _MM,)
        cmd = fields.get("cmd")
        if cmd is not None:
            flags |= EXEC_HAS_CMD
        if fields.get("held"):
            flags |= EXEC_HELD
        if fields.get("latched"):
            flags |= EXEC_LATCHED
        if fields.get("stopping"):
            flags |= EXEC_STOPPING
        n_qc, qc = _pad(fields.get("qc"), _DEG)
        body = struct.pack(
            EXEC_FMT, flags, MODE_CODES.get(mode, MODE_UNKNOWN), n_qc, 0,
            *target, *_pose_si(cmd), *qc,
            _num(fields.get("v")) * _MM, _num(fields.get("w")) * _DEG)
    elif kind == KIND_STATE:
        if len(item) == 3:                      # an ArmSnapshot (mm/deg)
            snap = item[2]
            joints = tuple(v * _DEG for v in snap.joints) if snap.joints else None
            speeds, tcp = None, _pose_si(snap.pose)
            robot_mode, safety_mode = "", ""
            arm_state, gripper_mm = int(snap.arm_state), snap.gripper
        else:
            (_, _, joints, speeds, tcp, robot_mode, safety_mode,
             arm_state, gripper_mm) = item
        n_q, q = _pad(joints)
        n_qd, qd = _pad(speeds)
        flags = (STATE_HAS_SPEEDS if n_qd else 0) | (STATE_HAS_TCP if tcp else 0)
        tcp_vals = tuple(float(v) for v in tcp) if tcp else (_NAN,) * 6
        body = struct.pack(
            STATE_FMT, n_q, flags, arm_state & 0xFF, 0, *q, *qd, *tcp_vals,
            _num(gripper_mm) * _MM, _ascii16(robot_mode), _ascii16(safety_mode))
    elif kind == KIND_CLOCK:
        _, _, offset_ns, rtt_us = item
        body = struct.pack(CLOCK_FMT, int(offset_ns), int(rtt_us) & 0xFFFFFFFF)
    else:
        raise ValueError(f"unknown capture record kind {kind}")
    return header + body


def unpack_record(data: bytes) -> dict:
    """Inverse of pack_record, for tests and offline tools."""
    version, kind, _reserved, tap_seq, t_ns = struct.unpack_from(HEADER_FMT, data)
    out: dict = {"version": version, "kind": kind, "tap_seq": tap_seq, "t_mono_ns": t_ns}
    body = data[HEADER_SIZE:]
    if kind == KIND_RX:
        v = struct.unpack(RX_FMT, body)
        out.update(hb_version=v[0], flags=v[1], time_flags=v[2], reason=v[3],
                   session_id=v[4], seq=v[5], motion_permit_id=v[6],
                   pose=v[7:13], gripper_m=v[13], pose_time_ns=v[14],
                   shown_capture_ns=(v[15], v[16]))
    elif kind == KIND_EXEC:
        v = struct.unpack(EXEC_FMT, body)
        out.update(flags=v[0], mode=v[1], n_qc=v[2], target=v[4:11], cmd=v[11:17],
                   qc=v[17:17 + v[2]], v_m_s=v[24], w_rad_s=v[25])
    elif kind == KIND_STATE:
        v = struct.unpack(STATE_FMT, body)
        out.update(n_q=v[0], flags=v[1], arm_state=v[2], joints=v[4:4 + v[0]],
                   speeds=v[11:18], tcp=v[18:24], gripper_m=v[24],
                   robot_mode=v[25].rstrip(b"\0").decode("ascii"),
                   safety_mode=v[26].rstrip(b"\0").decode("ascii"))
    elif kind == KIND_CLOCK:
        offset_ns, rtt_us = struct.unpack(CLOCK_FMT, body)
        out.update(offset_ns=offset_ns, rtt_us=rtt_us)
    return out
