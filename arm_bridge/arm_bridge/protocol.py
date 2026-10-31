"""Wire codec for the Quest <-> server arm protocol.

Implements the byte layouts in quest-bridge-interfacing-v3.md sections 4
(UDP/DataChannel `arm` heartbeat) and 5 (TCP/DataChannel `armctl` control +
status). Every field is little-endian; one serializer covers the whole protocol
(rev 3 section 2).

This module is transport-agnostic: it turns bytes <-> typed messages and does
*structural* validation only (magic, version, length). Semantic validation
(finite values, workspace/velocity limits, session/seq freshness) lives in
safety.py and session.py, so the same codec serves both the server and a test
peer that legitimately encodes edge-case values.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

# ---- constants (rev 3 sections 4 and 5) ----------------------------------
# Wire rev 0x03 (TURN Phase 1, Increment C): adds a u32 motionPermitId to the
# heartbeat (after seq) and STATUS (at the end) for command-freshness without a
# synced clock (§5.6). Coordinated Quest/server bump; old peers fail closed.
HEARTBEAT_MAGIC = 0xA5
HEARTBEAT_VERSION = 0x03
HEARTBEAT_SIZE = 44
# Wire rev 0x04 (CAPTURE-SYNC-QUEST-HANDOFF.md §2): rev 0x03 + the controller
# pose time (Quest clock) and, per session camera, the station capture time of
# the frame the Quest has on screen. Telemetry for the dataset only -- nothing
# in it reaches a motion decision. Sent by a Quest only after a HELLO reply of
# 4; the bridge decodes both revs from then on.
HEARTBEAT_VERSION_V4 = 0x04
HEARTBEAT_SIZE_V4 = 68

CONTROL_MAGIC = 0xB5
CONTROL_VERSION = 0x03
CONTROL_HEADER_SIZE = 12
CONTROL_PAYLOAD_MAX = 1024  # rev 3 section 5: bound payloadLen before allocating

# Protocol version negotiated in the HELLO reply code (Quest checks it). The
# bridge replies `protocol_version` from its config (default 3); 4 tells the
# Quest to send rev 0x04 heartbeats. Today's Quest accepts exactly 3, so 4 is
# switched on only after every headset accepts both (handoff §2 rollout).
PROTOCOL_VERSION = 3
PROTOCOL_VERSIONS = (3, 4)

STATUS_PAYLOAD_SIZE = 44

# heartbeat flags
FLAG_DEADMAN = 0x01  # bit0: operator holding clutch, drive toward target

# control message types
TYPE_HELLO = 0x01
TYPE_SET_ENABLED = 0x02
TYPE_CLEAR_ERROR = 0x03
TYPE_PING = 0x04          # Quest -> server: arm-channel RTT probe (TURN Phase 1)
TYPE_ARM_ACTION = 0x05    # Quest -> server: one-shot arm control (enable/manual/home/recover)
TYPE_CLIENT_STATS = 0x06  # Quest -> server: ~1 Hz link/video/headset telemetry (display only, no reply)
TYPE_REPLY = 0x80
TYPE_PONG = 0x81          # server -> Quest: PING echo (requestId is the nonce)
TYPE_STATUS = 0x20

# ARM_ACTION action codes (u8 payload). One-shot, run on the servo thread and
# replied to like CLEAR_ERROR. Never renumber; only append.
ARM_ACTION_MOTION_ENABLE = 0   # motion_enable(True) + mode 0 + state 0 -> READY
ARM_ACTION_MOTION_DISABLE = 1  # motion_enable(False) -> motors off
ARM_ACTION_MANUAL_ON = 2       # enter manual/free-drive (mode 2)
ARM_ACTION_MANUAL_OFF = 3      # leave manual, back to position mode (mode 0)
ARM_ACTION_HOME = 4            # controlled move to the home pose
ARM_ACTION_RECOVER = 5         # clean_warn + clean_error + motion_enable + mode 0 + state 0
ARM_ACTION_REARM = 6           # manual re-arm/resume: re-enter control mode + state 0
                               # (mode 7 idles to state 5 MODE_CHANGED; this re-arms it)
ARM_ACTION_AUTOREARM_ON = 7    # servo loop re-arms automatically on idle (default)
ARM_ACTION_AUTOREARM_OFF = 8   # servo loop holds on idle; operator presses RESUME (REARM)
ARM_ACTION_HOME_CANCEL = 9     # cancel an in-flight HOME move (stop_motion); releases
                               # the home standdown so the loop re-enters control
ARM_ACTION_BACKOUT = 10        # clear the fault, then replay the recorded trail (~3 s)
                               # BACKWARDS (undo the motion that caused it); in joint
                               # space on backends that support it (AUBO: no IK, so it
                               # works at a singularity). Also valid with no fault
                               # (a "target held" warning). Clear-only after an e-stop
                               # (controller codes 1-3) or with an empty trail.
                               # Reported as HOMING while replaying.
ARM_ACTION_RETREAT = 11        # slow planned move back inside the envelope (warn 1006):
                               # the dashboard's Retreat. Reported as HOMING while it
                               # runs; HOME_CANCEL stops it. Refused when already inside.

# status flags
STATUS_FLAG_MOVING = 0x01
STATUS_FLAG_SERVO_ACTIVE = 0x02
STATUS_FLAG_PROTECTIVE_STOP = 0x04
# bit3: the session's comm/quality hold is latched (heartbeat watchdog gap or
# stale motion permit while the clutch was held; an out-of-box target does NOT
# trip it -- a rejected target still counts as liveness, the arm just holds at
# the last valid one).
# The servo loop refuses to drive until the operator releases the deadman and
# re-engages; surfaced so the client can show "server hold: release grip"
# instead of looking healthy while the arm silently ignores the clutch.
# Additive bit: wire rev stays 0x03 (an old peer just ignores it).
STATUS_FLAG_HOLD_LATCHED = 0x08
# bit4: errorCode is an xArm SDK **API return code** (latched ArmFault from a
# failed SDK call), NOT a controller error code from the robot report. The two
# tables collide numerically (controller 1 = e-stop button; API 1 = "errors not
# cleared"), so without this bit the client cannot label the code correctly.
# Additive; wire rev stays 0x03.
STATUS_FLAG_ERRCODE_IS_API = 0x10

# struct formats (little-endian)
_HEARTBEAT_FMT = "<4B3I7f"  # magic,version,flags,reserved | sessionId,seq,motionPermitId | x..gripper
# rev 0x04: byte 3 is timeFlags; then poseTimeNs, shownCaptureNs[0], shownCaptureNs[1]
_HEARTBEAT_V4_FMT = "<4B3I7f3Q"
# heartbeat timeFlags (rev 0x04, byte 3)
TIME_FLAG_POSE_XR_DISPLAY = 0x01  # poseTimeNs is the OpenXR predicted display time
# PONG payload (handoff §3): station CLOCK_MONOTONIC at PING receipt / PONG send
_PONG_FMT = "<QQ"
_CONTROL_HEADER_FMT = "<4BII"  # magic,version,type,flags | requestId,payloadLen
_STATUS_FMT = "<BBH7fiiI"  # armState,flags,reserved | x..gripper | errorCode,warnCode,motionPermitId

# CLIENT_STATS (0x06): a fixed block, then cameraCount camera blocks. The block
# sizes travel in the payload, so a later client can append fields to either
# block and an older server still reads the fields it knows (trailing bytes are
# skipped). Display-only telemetry: nothing in it ever reaches a motion decision.
CLIENT_STATS_VERSION = 1
CLIENT_STATS_FIXED_SIZE = 40
CLIENT_STATS_CAMERA_SIZE = 32
CLIENT_STATS_MAX_CAMERAS = 8
# Appended fields (CAPTURE-SYNC-QUEST-HANDOFF.md §3 / §6): fixed block +16
# (clockOffsetNs i64, clockOffsetRttUs u32, reserved u32), camera block +8
# (displayLatencyP50Ms, displayLatencyP95Ms). Read when the block sizes on the
# wire are large enough; no version bump.
CLIENT_STATS_FIXED_SIZE_V2 = 56
CLIENT_STATS_CAMERA_SIZE_V2 = 40
_CLIENT_STATS_FIXED_TAIL_FMT = "<qII"
_CLIENT_STATS_CAMERA_TAIL_FMT = "<ff"
# version,fixedLen,cameraCount,cameraLen | armPingRtt,armTransportRtt,
# videoTransportRtt,appFps | armPathChanges,armIceReconnects,statusStaleCount |
# videoLocalCand,videoRemoteCand,armLocalCand,armRemoteCand,batteryPct,
# wifiRssiDbm(i8),thermalStatus,reserved | wifiLinkMbps,wifiFreqMhz,reserved
_CLIENT_STATS_FIXED_FMT = "<4B4f3H5BbBB3H"
# slot,state,idrRequests | fps,packetsReceived,packetsLost,lastPacketAgeMs,
# decodeP50Ms,decodeP95Ms | decoderResets,incompleteAuDrops
_CLIENT_STATS_CAMERA_FMT = "<BBHfIIfffHH"

# ICE candidate type codes in CLIENT_STATS (video/arm local/remote candidates).
# The Quest reports the arm pair because webrtcbin (1.24) emits no candidate-
# pair stats for a DataChannel-only PeerConnection: the station cannot see it.
CANDIDATE_TYPES = {0: None, 1: "host", 2: "srflx", 3: "prflx", 4: "relay"}
# Camera `state` codes: the Quest's StreamState enum, in order.
CAMERA_STATES = {0: "stopped", 1: "starting", 2: "waiting_rtp", 3: "waiting_config",
                 4: "waiting_idr", 5: "streaming", 6: "restarting_decoder", 7: "error"}


class ArmState(IntEnum):
    """rev 3 section 5 status `armState` enum."""
    UNKNOWN = 0
    DISABLED = 1
    ENABLING = 2
    READY = 3
    SERVO = 4
    HOMING = 5
    ERROR = 6
    PROTECTIVE_STOP = 7


class ProtocolError(ValueError):
    """A frame violated the wire format (magic/version/length). Callers drop
    the frame; they never move the arm on a malformed packet."""


@dataclass(frozen=True)
class Heartbeat:
    """A UDP/`arm` command packet: 44 bytes at wire rev 0x03 (rev 3 section 4),
    68 at rev 0x04. `motion_permit_id` echoes the newest STATUS permit (0 before
    one is seen). The rev 0x04 fields are dataset telemetry (0 = unknown):
    `pose_time_ns` in the Quest's CLOCK_MONOTONIC, `shown_capture_ns[i]` the
    station capture time of the frame on screen for session camera i."""
    flags: int
    session_id: int
    seq: int
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    gripper: float
    motion_permit_id: int = 0
    version: int = HEARTBEAT_VERSION
    time_flags: int = 0
    pose_time_ns: int = 0
    shown_capture_ns: tuple[int, int] = (0, 0)

    @property
    def deadman(self) -> bool:
        return bool(self.flags & FLAG_DEADMAN)

    @property
    def pose(self) -> tuple[float, float, float, float, float, float]:
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)


@dataclass(frozen=True)
class ControlFrame:
    """A 12-byte-header `armctl` message (rev 3 section 5)."""
    type: int
    request_id: int
    payload: bytes


@dataclass(frozen=True)
class Reply:
    """REPLY payload (rev 3 section 5): u8 ok, i32 code, u16 msgLen, UTF-8 msg."""
    request_id: int
    ok: bool
    code: int
    msg: str = ""


@dataclass(frozen=True)
class Status:
    """44-byte STATUS payload (rev 3 section 5 + wire rev 0x03). `motion_permit_id`
    is the rolling freshness token the Quest echoes in its heartbeat (§5.6)."""
    arm_state: ArmState
    moving: bool
    servo_active: bool
    protective_stop: bool
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    gripper: float
    error_code: int = 0
    warn_code: int = 0
    motion_permit_id: int = 0
    hold_latched: bool = False
    error_is_api: bool = False   # error_code is an SDK API code, not a controller code


_NAN = float("nan")


@dataclass(frozen=True)
class CameraStats:
    """One camera block of CLIENT_STATS: the Quest's receive side of stream
    `slot`. Counters are cumulative since that stream (re)started."""
    slot: int
    state: int = 0                    # CAMERA_STATES code
    idr_requests: int = 0             # keyframe requests (PLI) sent
    fps: float = _NAN                 # frames displayed per second
    packets_received: int = 0
    packets_lost: int = 0             # RTP sequence gaps
    last_packet_age_ms: float = _NAN  # NaN = no packet yet
    decode_p50_ms: float = _NAN       # decoder submit -> output
    decode_p95_ms: float = _NAN
    decoder_resets: int = 0
    incomplete_au_drops: int = 0
    # Appended (handoff §6): station capture -> latched on the Quest, ms.
    display_latency_p50_ms: float = _NAN
    display_latency_p95_ms: float = _NAN


@dataclass(frozen=True)
class ClientStats:
    """CLIENT_STATS payload: what the headset measures about its links, video
    and itself. NaN / 0 / 255 mean "unknown" per field (see the handoff doc)."""
    arm_ping_rtt_ms: float = _NAN         # last PING -> PONG on armctl
    arm_transport_rtt_ms: float = _NAN    # arm PeerConnection transport RTT
    video_transport_rtt_ms: float = _NAN  # video PeerConnection transport RTT
    app_fps: float = _NAN                 # headset frame rate
    arm_path_changes: int = 0
    arm_ice_reconnects: int = 0
    status_stale_count: int = 0           # STATUS went stale past the motion gate
    video_local_candidate: int = 0        # CANDIDATE_TYPES code
    video_remote_candidate: int = 0
    arm_local_candidate: int = 0          # the headset's side of the arm pair
    arm_remote_candidate: int = 0
    battery_pct: int = 255                # 255 = unknown
    wifi_rssi_dbm: int = 0                # 0 = unknown
    thermal_status: int = 255             # Android THERMAL_STATUS_*; 255 = unknown
    wifi_link_mbps: int = 0               # 0 = unknown
    wifi_freq_mhz: int = 0                # 0 = unknown
    cameras: tuple[CameraStats, ...] = ()
    # Appended (handoff §3): station - Quest clock offset from timestamped
    # PING/PONG, and the round trip of the exchange it came from. None / 0 =
    # no estimate yet (an older client, or before the first PONG).
    clock_offset_ns: int | None = None
    clock_offset_rtt_us: int = 0


# ---- heartbeat ------------------------------------------------------------
def decode_heartbeat(data: bytes) -> Heartbeat:
    """Rev 0x03 (44 bytes) or rev 0x04 (68 bytes), chosen by the version byte
    with the length checked. Anything else is malformed."""
    if len(data) not in (HEARTBEAT_SIZE, HEARTBEAT_SIZE_V4):
        raise ProtocolError(f"heartbeat must be {HEARTBEAT_SIZE} or {HEARTBEAT_SIZE_V4} "
                            f"bytes, got {len(data)}")
    magic, version = data[0], data[1]
    if magic != HEARTBEAT_MAGIC:
        raise ProtocolError(f"heartbeat magic {magic:#04x} != {HEARTBEAT_MAGIC:#04x}")
    if version == HEARTBEAT_VERSION and len(data) == HEARTBEAT_SIZE:
        (_magic, _version, flags, _reserved, session_id, seq, motion_permit_id,
         x, y, z, roll, pitch, yaw, gripper) = struct.unpack(_HEARTBEAT_FMT, data)
        return Heartbeat(flags, session_id, seq, x, y, z, roll, pitch, yaw, gripper,
                         motion_permit_id)
    if version == HEARTBEAT_VERSION_V4 and len(data) == HEARTBEAT_SIZE_V4:
        (_magic, _version, flags, time_flags, session_id, seq, motion_permit_id,
         x, y, z, roll, pitch, yaw, gripper,
         pose_time_ns, shown0, shown1) = struct.unpack(_HEARTBEAT_V4_FMT, data)
        return Heartbeat(flags, session_id, seq, x, y, z, roll, pitch, yaw, gripper,
                         motion_permit_id, HEARTBEAT_VERSION_V4, time_flags,
                         pose_time_ns, (shown0, shown1))
    raise ProtocolError(f"heartbeat version {version:#04x} with {len(data)} bytes "
                        f"(want {HEARTBEAT_VERSION:#04x}/{HEARTBEAT_SIZE} or "
                        f"{HEARTBEAT_VERSION_V4:#04x}/{HEARTBEAT_SIZE_V4})")


def encode_heartbeat(hb: Heartbeat) -> bytes:
    """Encode at `hb.version` (0x03 by default, 0x04 with the telemetry tail)."""
    if hb.version == HEARTBEAT_VERSION_V4:
        return struct.pack(
            _HEARTBEAT_V4_FMT,
            HEARTBEAT_MAGIC, HEARTBEAT_VERSION_V4, hb.flags, hb.time_flags & 0xFF,
            hb.session_id & 0xFFFFFFFF, hb.seq & 0xFFFFFFFF, hb.motion_permit_id & 0xFFFFFFFF,
            hb.x, hb.y, hb.z, hb.roll, hb.pitch, hb.yaw, hb.gripper,
            hb.pose_time_ns, hb.shown_capture_ns[0], hb.shown_capture_ns[1],
        )
    return struct.pack(
        _HEARTBEAT_FMT,
        HEARTBEAT_MAGIC, HEARTBEAT_VERSION, hb.flags, 0,
        hb.session_id & 0xFFFFFFFF, hb.seq & 0xFFFFFFFF, hb.motion_permit_id & 0xFFFFFFFF,
        hb.x, hb.y, hb.z, hb.roll, hb.pitch, hb.yaw, hb.gripper,
    )


# ---- control frames -------------------------------------------------------
def decode_control_frame(data: bytes) -> ControlFrame:
    if len(data) < CONTROL_HEADER_SIZE:
        raise ProtocolError(f"control frame shorter than {CONTROL_HEADER_SIZE}-byte header")
    magic, version, type_, _flags, request_id, payload_len = \
        struct.unpack(_CONTROL_HEADER_FMT, data[:CONTROL_HEADER_SIZE])
    if magic != CONTROL_MAGIC:
        raise ProtocolError(f"control magic {magic:#04x} != {CONTROL_MAGIC:#04x}")
    if version != CONTROL_VERSION:
        raise ProtocolError(f"control version {version:#04x} != {CONTROL_VERSION:#04x}")
    if payload_len > CONTROL_PAYLOAD_MAX:
        raise ProtocolError(f"payloadLen {payload_len} exceeds {CONTROL_PAYLOAD_MAX}")
    payload = data[CONTROL_HEADER_SIZE:CONTROL_HEADER_SIZE + payload_len]
    if len(payload) != payload_len:
        raise ProtocolError(f"control payload truncated: want {payload_len}, got {len(payload)}")
    return ControlFrame(type_, request_id, payload)


def encode_control_frame(type_: int, request_id: int, payload: bytes = b"") -> bytes:
    header = struct.pack(_CONTROL_HEADER_FMT, CONTROL_MAGIC, CONTROL_VERSION,
                         type_, 0, request_id & 0xFFFFFFFF, len(payload))
    return header + payload


def decode_hello(payload: bytes) -> int:
    if len(payload) != 4:
        raise ProtocolError("HELLO payload must be u32 sessionId")
    return struct.unpack("<I", payload)[0]


def decode_set_enabled(payload: bytes) -> bool:
    if len(payload) != 1:
        raise ProtocolError("SET_ENABLED payload must be u8 enabled")
    return payload[0] != 0


def decode_arm_action(payload: bytes) -> int:
    if len(payload) != 1:
        raise ProtocolError("ARM_ACTION payload must be u8 action")
    return payload[0]


def encode_arm_action(request_id: int, action: int) -> bytes:
    return encode_control_frame(TYPE_ARM_ACTION, request_id, struct.pack("<B", action & 0xFF))


def encode_hello(request_id: int, session_id: int) -> bytes:
    return encode_control_frame(TYPE_HELLO, request_id, struct.pack("<I", session_id & 0xFFFFFFFF))


def encode_set_enabled(request_id: int, enabled: bool) -> bytes:
    return encode_control_frame(TYPE_SET_ENABLED, request_id, struct.pack("<B", 1 if enabled else 0))


def encode_clear_error(request_id: int) -> bytes:
    return encode_control_frame(TYPE_CLEAR_ERROR, request_id, b"")


def encode_ping(request_id: int) -> bytes:
    """Arm-channel RTT probe (TURN Phase 1). Empty payload; the requestId is the
    correlation nonce the server echoes in the PONG. Append-only: an old server
    ignores the unknown type, so no version bump."""
    return encode_control_frame(TYPE_PING, request_id, b"")


def encode_pong(request_id: int, station_rx_ns: int | None = None,
                station_tx_ns: int | None = None) -> bytes:
    """Echo of a PING's requestId so the Quest can time the arm-control round
    trip. Carries no session/servo interaction. With the station's receipt and
    send times (CLOCK_MONOTONIC ns) the Quest also estimates the clock offset
    (handoff §3); an older Quest ignores the payload."""
    if station_rx_ns is None or station_tx_ns is None:
        return encode_control_frame(TYPE_PONG, request_id, b"")
    return encode_control_frame(TYPE_PONG, request_id,
                                struct.pack(_PONG_FMT, station_rx_ns, station_tx_ns))


def decode_pong_times(payload: bytes) -> tuple[int, int] | None:
    """(station_rx_ns, station_tx_ns) from a PONG payload, None when empty
    (an older bridge). Tests and the test peer use it."""
    if not payload:
        return None
    if len(payload) < struct.calcsize(_PONG_FMT):
        raise ProtocolError("PONG payload shorter than 16 bytes")
    return struct.unpack_from(_PONG_FMT, payload)


# ---- client stats ----------------------------------------------------------
def _u16(value: int) -> int:
    return max(0, min(int(value), 0xFFFF))   # cumulative counters saturate


def encode_client_stats(stats: ClientStats, extended: bool = False) -> bytes:
    """CLIENT_STATS frame (requestId 0: unsolicited, never replied to). The
    Quest is the real sender; this encoder serves tests and the test peer.
    `extended` writes the appended capture-sync fields (56/40-byte blocks)."""
    cams = stats.cameras[:CLIENT_STATS_MAX_CAMERAS]
    fixed_size = CLIENT_STATS_FIXED_SIZE_V2 if extended else CLIENT_STATS_FIXED_SIZE
    cam_size = CLIENT_STATS_CAMERA_SIZE_V2 if extended else CLIENT_STATS_CAMERA_SIZE
    payload = struct.pack(
        _CLIENT_STATS_FIXED_FMT,
        CLIENT_STATS_VERSION, fixed_size, len(cams), cam_size,
        stats.arm_ping_rtt_ms, stats.arm_transport_rtt_ms, stats.video_transport_rtt_ms,
        stats.app_fps,
        _u16(stats.arm_path_changes), _u16(stats.arm_ice_reconnects),
        _u16(stats.status_stale_count),
        stats.video_local_candidate & 0xFF, stats.video_remote_candidate & 0xFF,
        stats.arm_local_candidate & 0xFF, stats.arm_remote_candidate & 0xFF,
        stats.battery_pct & 0xFF, max(-128, min(int(stats.wifi_rssi_dbm), 127)),
        stats.thermal_status & 0xFF, 0,
        _u16(stats.wifi_link_mbps), _u16(stats.wifi_freq_mhz), 0,
    )
    if extended:
        rtt_us = stats.clock_offset_rtt_us if stats.clock_offset_ns is not None else 0
        payload += struct.pack(_CLIENT_STATS_FIXED_TAIL_FMT, stats.clock_offset_ns or 0,
                               rtt_us & 0xFFFFFFFF, 0)
    for cam in cams:
        payload += struct.pack(
            _CLIENT_STATS_CAMERA_FMT,
            cam.slot & 0xFF, cam.state & 0xFF, _u16(cam.idr_requests),
            cam.fps, cam.packets_received & 0xFFFFFFFF, cam.packets_lost & 0xFFFFFFFF,
            cam.last_packet_age_ms, cam.decode_p50_ms, cam.decode_p95_ms,
            _u16(cam.decoder_resets), _u16(cam.incomplete_au_drops),
        )
        if extended:
            payload += struct.pack(_CLIENT_STATS_CAMERA_TAIL_FMT,
                                   cam.display_latency_p50_ms, cam.display_latency_p95_ms)
    return encode_control_frame(TYPE_CLIENT_STATS, 0, payload)


def decode_client_stats(payload: bytes) -> ClientStats:
    """Parse a CLIENT_STATS payload. Reads the v1 fields of each block and
    skips any bytes a newer client appended (the block sizes are on the wire)."""
    if len(payload) < CLIENT_STATS_FIXED_SIZE:
        raise ProtocolError(f"CLIENT_STATS shorter than {CLIENT_STATS_FIXED_SIZE} bytes")
    (version, fixed_len, count, cam_len,
     ping_rtt, arm_rtt, video_rtt, app_fps,
     path_changes, ice_reconnects, stale_count,
     video_local, video_remote, arm_local, arm_remote, battery, rssi, thermal, _reserved,
     link_mbps, freq_mhz, _reserved2) = struct.unpack(
        _CLIENT_STATS_FIXED_FMT, payload[:CLIENT_STATS_FIXED_SIZE])
    if version < 1:
        raise ProtocolError(f"CLIENT_STATS version {version} < 1")
    if fixed_len < CLIENT_STATS_FIXED_SIZE:
        raise ProtocolError(f"CLIENT_STATS fixedLen {fixed_len} < {CLIENT_STATS_FIXED_SIZE}")
    if count > CLIENT_STATS_MAX_CAMERAS:
        raise ProtocolError(f"CLIENT_STATS cameraCount {count} > {CLIENT_STATS_MAX_CAMERAS}")
    if count and cam_len < CLIENT_STATS_CAMERA_SIZE:
        raise ProtocolError(f"CLIENT_STATS cameraLen {cam_len} < {CLIENT_STATS_CAMERA_SIZE}")
    if len(payload) < fixed_len + count * cam_len:
        raise ProtocolError("CLIENT_STATS truncated: camera blocks missing")
    cameras = []
    for i in range(count):
        start = fixed_len + i * cam_len
        fields_ = struct.unpack(
            _CLIENT_STATS_CAMERA_FMT, payload[start:start + CLIENT_STATS_CAMERA_SIZE])
        if cam_len >= CLIENT_STATS_CAMERA_SIZE_V2:
            fields_ += struct.unpack_from(_CLIENT_STATS_CAMERA_TAIL_FMT, payload,
                                          start + CLIENT_STATS_CAMERA_SIZE)
        cameras.append(CameraStats(*fields_))
    clock_offset_ns, clock_rtt_us = None, 0
    if fixed_len >= CLIENT_STATS_FIXED_SIZE_V2:
        offset, rtt_us, _reserved3 = struct.unpack_from(
            _CLIENT_STATS_FIXED_TAIL_FMT, payload, CLIENT_STATS_FIXED_SIZE)
        if rtt_us:                              # 0 = no estimate yet
            clock_offset_ns, clock_rtt_us = offset, rtt_us
    return ClientStats(
        ping_rtt, arm_rtt, video_rtt, app_fps,
        path_changes, ice_reconnects, stale_count,
        video_local, video_remote, arm_local, arm_remote, battery, rssi, thermal,
        link_mbps, freq_mhz, tuple(cameras), clock_offset_ns, clock_rtt_us,
    )


# ---- reply ----------------------------------------------------------------
def encode_reply(reply: Reply) -> bytes:
    msg_bytes = reply.msg.encode("utf-8")
    payload = struct.pack("<BiH", 1 if reply.ok else 0, reply.code, len(msg_bytes)) + msg_bytes
    return encode_control_frame(TYPE_REPLY, reply.request_id, payload)


def decode_reply(frame: ControlFrame) -> Reply:
    if frame.type != TYPE_REPLY:
        raise ProtocolError(f"not a REPLY frame (type {frame.type:#04x})")
    if len(frame.payload) < 7:
        raise ProtocolError("REPLY payload shorter than 7 bytes")
    ok, code, msg_len = struct.unpack("<BiH", frame.payload[:7])
    msg = frame.payload[7:7 + msg_len].decode("utf-8", errors="replace")
    if len(frame.payload) != 7 + msg_len:
        raise ProtocolError("REPLY payloadLen must equal 7 + msgLen")
    return Reply(frame.request_id, bool(ok), code, msg)


# ---- status ---------------------------------------------------------------
def encode_status(status: Status) -> bytes:
    flags = 0
    if status.moving:
        flags |= STATUS_FLAG_MOVING
    if status.servo_active:
        flags |= STATUS_FLAG_SERVO_ACTIVE
    if status.protective_stop:
        flags |= STATUS_FLAG_PROTECTIVE_STOP
    if status.hold_latched:
        flags |= STATUS_FLAG_HOLD_LATCHED
    if status.error_is_api:
        flags |= STATUS_FLAG_ERRCODE_IS_API
    payload = struct.pack(
        _STATUS_FMT,
        int(status.arm_state), flags, 0,
        status.x, status.y, status.z, status.roll, status.pitch, status.yaw, status.gripper,
        status.error_code, status.warn_code, status.motion_permit_id & 0xFFFFFFFF,
    )
    # STATUS always uses requestId = 0 (rev 3 section 5).
    return encode_control_frame(TYPE_STATUS, 0, payload)


def decode_status(frame: ControlFrame) -> Status:
    if frame.type != TYPE_STATUS:
        raise ProtocolError(f"not a STATUS frame (type {frame.type:#04x})")
    if len(frame.payload) != STATUS_PAYLOAD_SIZE:
        raise ProtocolError(f"STATUS payload must be {STATUS_PAYLOAD_SIZE} bytes")
    (arm_state, flags, _reserved, x, y, z, roll, pitch, yaw, gripper,
     error_code, warn_code, motion_permit_id) = struct.unpack(_STATUS_FMT, frame.payload)
    return Status(
        ArmState(arm_state),
        bool(flags & STATUS_FLAG_MOVING),
        bool(flags & STATUS_FLAG_SERVO_ACTIVE),
        bool(flags & STATUS_FLAG_PROTECTIVE_STOP),
        x, y, z, roll, pitch, yaw, gripper, error_code, warn_code, motion_permit_id,
        bool(flags & STATUS_FLAG_HOLD_LATCHED),
        bool(flags & STATUS_FLAG_ERRCODE_IS_API),
    )
