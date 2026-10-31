"""WebRTC DataChannel transport (answerer) for the arm bridge.

The arm rides its OWN PeerConnection, separate from the streamer's video one, so
video and arm stay independent failure domains (rev 3 sections 1/8; resolves the
Path-C v2 "coupling" question toward independence). The peer (Quest / test peer)
offers and opens two DataChannels; this side answers and receives them:

  * `arm`    — unreliable/unordered heartbeat (rev 3 section 4), 44-byte packets
  * `armctl` — reliable/ordered control + status (rev 3 section 5)

The reliability is configured by the offerer when it creates the channels; the
answerer just accepts whatever was negotiated. Incoming bytes are decoded and
handed to the Session (thread-safe); REPLY/STATUS go back out on `armctl`. The
servo loop is the only arm actuator and runs on its own thread, so nothing here
can block it (rev 3 section 10).

webrtcbin runs its own GMainContext/thread for ICE (like the C++ streamer), so
no app-level GLib main loop is needed; signaling actions are emitted directly
from the signaling thread.
"""

from __future__ import annotations

import logging
import socket
import threading
import time

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstWebRTC", "1.0")
gi.require_version("GstSdp", "1.0")
from gi.repository import GLib, Gst, GstSdp, GstWebRTC  # noqa: E402

from . import protocol
from .config import ArmBridgeConfig
from .servo import ServoLoop
from .session import ControlResult, PendingAction, PendingClear, Session

# `arm_bridge.client.*` routes to the dashboard's `client` log source via
# BusLogHandler; plain transport/lifecycle lines are `system`. Both print to
# the same stdout journal (logbus.wire_logging).
log = logging.getLogger("arm_bridge.transport")
client_log = logging.getLogger("arm_bridge.client.transport")


# ---- Server-reflexive candidate synthesis (WAN multi-homed workaround) --------
# Ported verbatim from the C++ streamer (streamer/src/webrtc_session.cpp): on this
# station libnice fails to gather ANY srflx candidate -- it tries every interface
# (Tailscale/docker0/IPv6 link-local), the Tailscale one times out against the
# public STUN server, and gathering ends host-only. So we learn the public mapping
# ourselves (one STUN query from the internet-facing interface) and, for the LAN
# host candidate webrtcbin emits, ALSO hand the peer a matching `typ srflx`. The
# station NAT is port-preserving, so the peer's checks to <public>:<port> land on
# libnice's real host socket at <lan>:<port>, which answers via peer-reflexive.
# The arm bridge needs this exactly like the streamer, or its PeerConnection fails
# ICE over WAN (host-only candidates the peer can't reach).

def _default_route_local_ipv4() -> str:
    """The internet-facing local IPv4, via a connected (never-sent) UDP socket."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 53))   # routable, never contacted
        return sock.getsockname()[0]
    except OSError:
        return ""
    finally:
        sock.close()


def _stun_mapped_ipv4(lan_ip: str, stun_host_port: str) -> str:
    """One STUN Binding request from lan_ip to stun_host_port ("host:port");
    returns the mapped public IPv4, or "" on any failure/timeout."""
    colon = stun_host_port.rfind(":")
    if colon < 0:
        return ""
    host, port = stun_host_port[:colon], stun_host_port[colon + 1:]
    try:
        infos = socket.getaddrinfo(host, port, socket.AF_INET, socket.SOCK_DGRAM)
    except OSError:
        return ""
    if not infos:
        return ""
    addr = infos[0][4]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        try:
            sock.bind((lan_ip, 0))
        except OSError:
            pass
        sock.settimeout(2.0)
        req = bytes((
            0x00, 0x01, 0x00, 0x00,              # Binding Request, length 0
            0x21, 0x12, 0xA4, 0x42,              # magic cookie
            0x01, 0x23, 0x45, 0x67, 0x89, 0xAB,  # transaction id (fixed is fine)
            0xCD, 0xEF, 0x00, 0x11, 0x22, 0x33,
        ))
        sock.sendto(req, addr)
        buf = sock.recv(512)
    except OSError:
        return ""
    finally:
        sock.close()
    if len(buf) < 20:
        return ""
    off, total = 20, len(buf)
    while off + 4 <= total:
        atype = (buf[off] << 8) | buf[off + 1]
        alen = (buf[off + 2] << 8) | buf[off + 3]
        if off + 4 + alen > total:
            break
        value = buf[off + 4:]
        if atype in (0x0020, 0x0001) and alen >= 8:
            if atype == 0x0020:   # XOR-MAPPED-ADDRESS
                octets = bytes((value[4] ^ 0x21, value[5] ^ 0x12,
                                value[6] ^ 0xA4, value[7] ^ 0x42))
            else:                 # MAPPED-ADDRESS
                octets = bytes((value[4], value[5], value[6], value[7]))
            return socket.inet_ntop(socket.AF_INET, octets)
        off += 4 + alen + ((4 - (alen % 4)) % 4)
    return ""


def _srflx_from_host_candidate(cand: str, lan_ip: str, public_ip: str) -> str:
    """If `cand` is a UDP host candidate on lan_ip, return an equivalent
    `typ srflx` line at public_ip with the same port (port-preserving NAT).
    Else "". candidate:<foundation> <comp> <proto> <prio> <ip> <port> typ host ..."""
    f = cand.split()
    if len(f) < 8 or f[2].lower() != "udp" or f[4] != lan_ip \
            or f[6] != "typ" or f[7] != "host":
        return ""
    comp, port = f[1], f[5]
    try:
        c = int(comp)
    except ValueError:
        c = 1
    if c <= 0:
        c = 1
    # srflx priority (RFC 5245): (type_pref 100)<<24 | (local_pref)<<8 | (256-comp).
    prio = (100 << 24) | (65535 << 8) | (256 - c)
    return (f"candidate:s{c} {comp} UDP {prio} {public_ip} {port} "
            f"typ srflx raddr {lan_ip} rport {port}")


class WebRtcArmTransport:
    def __init__(self, config: ArmBridgeConfig, session: Session, servo: ServoLoop,
                 signaling, state_hub=None, capture_tap=None) -> None:
        self._cfg = config
        self._session = session
        self._servo = servo
        self._signaling = signaling
        self._hub = state_hub    # StateHub | None -- dashboard state producer
        # ArmCaptureTap | None -- dataset hand-off (capture_tap.py): O(1)
        # appends on this transport thread, never a socket write here.
        self._capture = capture_tap
        self._last_deadman: bool | None = None  # client deadman edge-log state
        self._peer_token = ""
        self._mid_by_index: list[str] = []
        self._arm_ch = None      # heartbeat channel
        self._armctl_ch = None   # control/status channel
        self._ch_lock = threading.Lock()
        self._status_stop = threading.Event()
        self._status_thread: threading.Thread | None = None
        self._telemetry_next = 0.0   # next Phase 1 telemetry log (~1 Hz)
        # Last ASYNC get-stats result ("localType|remoteType|transport" or "").
        # Written from webrtcbin's reply thread, read on the status thread; a
        # str ref swap is GIL-atomic. Never fetched synchronously there.
        self._selected_pair_cache = ""
        self._pose_log_next = 0.0    # next client pose-command debug log (~5 Hz)
        self._last_status_key = None  # (arm_state, servo_active, protective_stop, hold) last logged
        self._reject_log_next = 0.0  # next heartbeat-rejection log (~1 Hz)
        self._rejects_since_log = 0  # rejections swallowed since the last log line

        Gst.init(None)
        self._pipe = None
        self._webrtc = None

        # WAN srflx-synthesis workaround (see the module note above; mirrors the
        # C++ streamer). Learn our public mapping once at construct so
        # _on_ice_candidate can hand the peer a srflx for each LAN host candidate.
        self._lan_ipv4 = ""
        self._public_ipv4 = ""
        if config.stun_server:
            self._lan_ipv4 = _default_route_local_ipv4()
            if self._lan_ipv4:
                self._public_ipv4 = _stun_mapped_ipv4(self._lan_ipv4, config.stun_server)
            log.info("arm bridge: srflx synth lan=%s public=%s",
                     self._lan_ipv4 or "?", self._public_ipv4 or "?")

    # ---- lifecycle --------------------------------------------------------
    def start(self) -> None:
        self._signaling.set_handlers(self._on_signal, self._on_disconnect, self._on_peer)
        self._status_thread = threading.Thread(target=self._status_loop, name="arm-status", daemon=True)
        self._status_thread.start()

    def stop(self) -> None:
        self._status_stop.set()
        if self._status_thread is not None:
            self._status_thread.join(timeout=2.0)
        self._teardown_pipeline()

    def _build_pipeline(self) -> None:
        pipe = Gst.Pipeline.new("arm-bridge")
        webrtc = Gst.ElementFactory.make("webrtcbin", "arm")
        webrtc.set_property("bundle-policy", GstWebRTC.WebRTCBundlePolicy.MAX_BUNDLE)
        if self._cfg.stun_server:
            # Config carries the canonical bare host:port (see config.py); the
            # stun:// scheme webrtcbin wants is added here, at the point of use.
            # Empty = LAN mode: host candidates only, exactly as before (S1).
            webrtc.set_property("stun-server", f"stun://{self._cfg.stun_server}")
        pipe.add(webrtc)
        webrtc.connect("on-ice-candidate", self._on_ice_candidate)
        webrtc.connect("on-data-channel", self._on_data_channel)
        # Transport-loss detection (safety review): a dead PeerConnection while the
        # signaling socket stays up must still invalidate the session and hold. The
        # 150 ms heartbeat watchdog already stops motion when `arm` goes silent;
        # this adds the documented session invalidation + defense in depth.
        webrtc.connect("notify::connection-state", self._on_conn_state)
        pipe.set_state(Gst.State.PLAYING)
        pipe.get_state(5 * Gst.SECOND)
        self._pipe = pipe
        self._webrtc = webrtc

    def _teardown_pipeline(self) -> None:
        pipe = self._pipe
        self._pipe = None
        self._webrtc = None
        with self._ch_lock:
            self._arm_ch = None
            self._armctl_ch = None
        self._mid_by_index = []
        if pipe is not None:
            pipe.set_state(Gst.State.NULL)

    # ---- signaling (answerer) --------------------------------------------
    def _on_peer(self, peer_ip: str) -> None:
        # One PeerConnection == one DTLS-bound peer, so the DTLS session is the
        # source binding the UDP spec approximates with source-IP checks. DTLS
        # encrypts/binds the session; it does NOT authenticate an authorized
        # operator (signaling is plain ws) -- run on a trusted LAN or tunnel.
        self._peer_token = peer_ip or "webrtc"
        # webrtcbin is single-session: a used instance cannot negotiate a second
        # independent DTLS session (session 2 stalls in ICE), so every peer gets
        # a fresh pipeline — same per-session pattern as the C++ streamer.
        self._teardown_pipeline()
        self._build_pipeline()

    def _on_signal(self, message: dict) -> None:
        kind = message.get("type")
        if kind == "hello":
            # cam mids are the streamer's concern; the arm advertises its channels.
            self._signaling.send({"type": "welcome", "channels": ["arm", "armctl"]})
        elif kind == "offer":
            if self._webrtc is not None:
                self._on_offer(message.get("sdp", ""))
        elif kind == "candidate":
            cand = message.get("candidate", "")
            if cand and self._webrtc is not None:
                mid = message.get("mid", "")
                if mid in self._mid_by_index:
                    self._webrtc.emit("add-ice-candidate", self._mid_by_index.index(mid), cand)
        elif kind == "bye":
            self._on_disconnect()

    def _on_offer(self, sdp_text: str) -> None:
        self._mid_by_index = [
            line[len("a=mid:"):].strip()
            for line in sdp_text.splitlines() if line.startswith("a=mid:")
        ]
        _res, sdp_msg = GstSdp.SDPMessage.new()
        GstSdp.sdp_message_parse_buffer(sdp_text.encode("utf-8"), sdp_msg)
        offer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.OFFER, sdp_msg)
        promise = Gst.Promise.new_with_change_func(self._on_remote_set, None)
        self._webrtc.emit("set-remote-description", offer, promise)

    def _on_remote_set(self, _promise, _user) -> None:
        promise = Gst.Promise.new_with_change_func(self._on_answer_created, None)
        self._webrtc.emit("create-answer", None, promise)

    def _on_answer_created(self, promise, _user) -> None:
        reply = promise.get_reply()
        if reply is None:
            return
        answer = reply.get_value("answer")
        if answer is None or answer.sdp is None:
            return
        sdp_text = answer.sdp.as_text()  # read before set-local consumes ownership
        self._webrtc.emit("set-local-description", answer, Gst.Promise.new())
        self._signaling.send({"type": "answer", "sdp": sdp_text})
        del reply

    def _on_ice_candidate(self, _element, mline_index, candidate) -> None:
        mid = self._mid_by_index[mline_index] if mline_index < len(self._mid_by_index) else ""
        self._signaling.send({"type": "candidate", "candidate": candidate, "mid": mid})
        # WAN workaround: libnice gathered no srflx here, so for each LAN host
        # candidate also hand the peer a synthesized srflx at our public mapping.
        if self._public_ipv4 and self._lan_ipv4:
            srflx = _srflx_from_host_candidate(candidate, self._lan_ipv4, self._public_ipv4)
            if srflx:
                log.info("arm bridge: inject srflx mid='%s' cand=%s", mid, srflx)
                self._signaling.send({"type": "candidate", "candidate": srflx, "mid": mid})

    def _on_disconnect(self) -> None:
        self._session.on_disconnect()
        self._teardown_pipeline()

    # ---- data channels ----------------------------------------------------
    def _on_data_channel(self, _element, channel) -> None:
        label = channel.get_property("label")
        log.info("arm bridge: data channel '%s' opened by peer", label)
        with self._ch_lock:
            if label == "arm":
                self._arm_ch = channel
                channel.connect("on-message-data", self._on_arm_data)
            elif label == "armctl":
                self._armctl_ch = channel
                channel.connect("on-message-data", self._on_armctl_data)
            # unknown labels are ignored (fail closed, don't crash)
        # Either channel closing/erroring invalidates the session and holds; both
        # ride one SCTP association, so a real loss takes them down together.
        channel.connect("on-close", self._on_channel_gone)
        channel.connect("on-error", self._on_channel_error)

    def _on_channel_gone(self, channel) -> None:
        label = channel.get_property("label")
        self._invalidate_session_hold(f"datachannel '{label}' closed")

    def _on_channel_error(self, channel, _error) -> None:
        label = channel.get_property("label")
        self._invalidate_session_hold(f"datachannel '{label}' error")

    def _on_conn_state(self, webrtc, _pspec) -> None:
        state = webrtc.get_property("connection-state")
        dead = (
            GstWebRTC.WebRTCPeerConnectionState.FAILED,
            GstWebRTC.WebRTCPeerConnectionState.DISCONNECTED,
            GstWebRTC.WebRTCPeerConnectionState.CLOSED,
        )
        if state in dead:
            self._invalidate_session_hold(f"peerconnection {state.value_nick}")

    def _invalidate_session_hold(self, reason: str) -> None:
        """Hold the arm on transport loss: invalidate the session (fresh HELLO
        required to resume). Deliberately does NOT tear down the pipeline from this
        GStreamer callback thread -- teardown happens on signaling drop / next peer;
        the safety-critical action here is the session hold."""
        log.warning("arm bridge: transport loss (%s); invalidating session", reason)
        self._session.on_disconnect()

    def _on_arm_data(self, _channel, gbytes) -> None:
        rx_ns = time.monotonic_ns()   # earliest receipt time the bridge sees
        data = gbytes.get_data()
        try:
            hb = protocol.decode_heartbeat(data)
        except protocol.ProtocolError:
            return  # drop malformed heartbeat, never move on it
        # Debug: client pose commands, throttled (~5 Hz) so this never floods
        # the log or adds latency on the heartbeat path -- heartbeats arrive
        # far faster than this.
        now = time.monotonic()
        if now >= self._pose_log_next:
            self._pose_log_next = now + 0.2
            client_log.info(
                "arm: pose cmd seq=%d deadman=%d "
                "pose=(%.1f,%.1f,%.1f,%.1f,%.1f,%.1f) gripper=%.1f",
                hb.seq, hb.deadman, hb.x, hb.y, hb.z,
                hb.roll, hb.pitch, hb.yaw, hb.gripper,
            )
        accepted, reason = self._session.on_heartbeat(hb, self._peer_token)
        if self._capture is not None:
            self._capture.heartbeat(rx_ns, hb, reason)
        if accepted:
            # Deadman edges are the client events that matter (plan §4:
            # heartbeats are edge-logged, never streamed at 50 Hz).
            deadman = bool(hb.deadman)
            if deadman != self._last_deadman:
                self._last_deadman = deadman
                client_log.info("arm: deadman %s",
                                "engaged" if deadman else "released")
        if not accepted:
            # A rejected heartbeat does NOT refresh the watchdog, so a sustained
            # rejection (e.g. target outside the workspace box) turns into a
            # silent hold latch. Log the reason, throttled to ~1 Hz with a count
            # of suppressed lines, so the cause is visible in the journal.
            self._rejects_since_log += 1
            if now >= self._reject_log_next:
                self._reject_log_next = now + 1.0
                client_log.warning(
                    "arm: heartbeat REJECTED (%s) seq=%d deadman=%d "
                    "pose=(%.1f,%.1f,%.1f,%.1f,%.1f,%.1f) [%d since last log]",
                    reason, hb.seq, hb.deadman, hb.x, hb.y, hb.z,
                    hb.roll, hb.pitch, hb.yaw, self._rejects_since_log,
                )
                self._rejects_since_log = 0

    def _on_armctl_data(self, _channel, gbytes) -> None:
        rx_ns = time.monotonic_ns()   # PING receipt time for the PONG (handoff §3)
        data = gbytes.get_data()
        try:
            frame = protocol.decode_control_frame(data)
        except protocol.ProtocolError:
            return
        if frame.type == protocol.TYPE_PING:
            # Echo immediately for the operator's arm-channel RTT display. Pure
            # transport echo -- no session/servo interaction, so it can never
            # affect motion. Reliable/ordered armctl, same as REPLY/STATUS. The
            # receipt/send times let the Quest estimate the clock offset; an
            # older Quest ignores them.
            self._send_ctl(protocol.encode_pong(frame.request_id, rx_ns,
                                                time.monotonic_ns()))
            return
        if frame.type == protocol.TYPE_CLIENT_STATS:
            # The headset's link/video telemetry for the dashboard. Display
            # only: it lands in the LinkMonitor and nowhere near the session's
            # intent or the servo loop. Never replied to.
            self._on_client_stats(frame.payload, rx_ns)
            return
        result = self._route_control(frame)
        if result is not None:
            reply = protocol.Reply(frame.request_id, result.ok, result.code, result.msg)
            self._send_ctl(protocol.encode_reply(reply))

    def _on_client_stats(self, payload: bytes, rx_ns: int = 0) -> None:
        try:
            stats = protocol.decode_client_stats(payload)
        except protocol.ProtocolError as exc:
            if not getattr(self, "_client_stats_err_logged", False):
                self._client_stats_err_logged = True
                client_log.warning("arm: malformed CLIENT_STATS dropped (once): %s", exc)
            return
        self._session.link.on_client_stats(stats)
        if self._capture is not None and stats.clock_offset_ns is not None:
            # The Quest's clock offset maps its pose times onto ours (dataset).
            self._capture.clock(rx_ns, stats.clock_offset_ns, stats.clock_offset_rtt_us)

    def _route_control(self, frame: protocol.ControlFrame) -> ControlResult | None:
        try:
            if frame.type == protocol.TYPE_HELLO:
                session_id = protocol.decode_hello(frame.payload)
                return self._session.on_hello(session_id, self._peer_token)
            if frame.type == protocol.TYPE_SET_ENABLED:
                enabled = protocol.decode_set_enabled(frame.payload)
                client_log.info("arm: SET_ENABLED received enabled=%s", enabled)
                return self._session.on_set_enabled(enabled)
            if frame.type == protocol.TYPE_CLEAR_ERROR:
                pending = self._session.request_clear_error()
                if isinstance(pending, PendingClear):
                    return pending.wait(1.0)   # servo loop fulfills it (rev 3 sec 5: ~1s)
                return pending
            if frame.type == protocol.TYPE_ARM_ACTION:
                action = protocol.decode_arm_action(frame.payload)
                client_log.info("arm: ARM_ACTION received action=%s", action)
                pending = self._session.request_arm_action(action)
                if isinstance(pending, PendingAction):
                    return pending.wait(3.0)   # enable/home may take longer than a clear
                return pending
        except protocol.ProtocolError as exc:
            return ControlResult(False, -1, str(exc))
        return None  # unknown type: no reply

    # ---- status ----------------------------------------------------------
    def _status_loop(self) -> None:
        """30 Hz STATUS to the Quest and -- same thread, same snapshot -- the
        dashboard StateHub publish. This thread is the hub's single producer
        (plan review): with a hub attached, status is built every period even
        with no Quest connected; without one, behavior is exactly as before."""
        period = 1.0 / self._cfg.status_hz
        while not self._status_stop.is_set():
            with self._ch_lock:
                ch = self._armctl_ch
            channel_open = ch is not None and self._channel_open(ch)
            if channel_open or self._hub is not None:
                st = self._servo.status()
                if self._hub is not None:
                    snap = self._servo.last_snapshot   # set by status(), same thread
                    self._hub.publish(st, self._session,
                                      joints=snap.joints if snap else None,
                                      envelope=getattr(self._servo, "envelope_block", None))
                if channel_open:
                    self._send_ctl(protocol.encode_status(st))
                    self._log_status_change(st)
                    self._log_telemetry(st)
            self._status_stop.wait(period)

    def _log_status_change(self, st) -> None:
        """Debug: log arm_state/servo_active/protective_stop transitions sent to
        the client, on change only -- to catch a STATUS flicker (e.g. a brief
        servo_active=false between an ARM_ACTION's set_mode(0) and the reconcile
        loop's set_mode(1)) that the client might read as an unexpected disable."""
        key = (st.arm_state, st.servo_active, st.protective_stop, st.hold_latched)
        if key != self._last_status_key:
            self._last_status_key = key
            log.info(
                "arm: status changed arm_state=%s servo_active=%s moving=%s "
                "protective_stop=%s hold_latched=%s err=%s warn=%s",
                st.arm_state.name, st.servo_active, st.moving,
                st.protective_stop, st.hold_latched, st.error_code, st.warn_code,
            )

    def _log_telemetry(self, status=None) -> None:
        """Phase 1 observability: emit heartbeat-freshness/gap counters plus the
        station's selected ICE candidate pair at ~1 Hz off the servo thread.
        Counters + candidate types only; no raw addresses (redaction §10)."""
        now = time.monotonic()
        if now < self._telemetry_next:
            return
        self._telemetry_next = now + 1.0
        s = self._session
        # Use the cached pair and kick off the next async fetch: this runs on
        # the STATUS thread, and a blocking get-stats here paused STATUS past
        # the Quest's 300 ms motion-freshness gate (~1 Hz "no servo status").
        path = self._selected_pair_cache
        self._request_selected_pair()
        arm_state = getattr(getattr(status, "arm_state", None), "name", "?")
        log.info(
            "arm telemetry: accepted=%s rejected=%s seq_skips=%s "
            "hb_gap_max_ms=%.0f hb_watchdog_gaps=%s "
            "permit_age_max_ms=%.0f permit_holds=%s "
            "arm_state=%s enabled=%s hold_latched=%s err=%s warn=%s%s",
            s.accepted_count, s.rejected_count, s.seq_skips,
            s.hb_gap_max_ms, s.hb_watchdog_gaps,
            s.permit_age_max_ms, s.permit_holds,
            arm_state, s.enabled_desired, s.hold_latched,
            getattr(status, "error_code", "?"), getattr(status, "warn_code", "?"),
            f" path={path}" if path else "",
        )

    def _request_selected_pair(self) -> None:
        """Kick off an ASYNC webrtcbin `get-stats`; the reply is parsed in
        `_on_stats_reply` (webrtcbin's thread) into `_selected_pair_cache` for the
        next telemetry line. Never waits: the previous synchronous
        `promise.wait()` ran on the STATUS thread, so a slow stats collection
        delayed STATUS and tripped the Quest's motion-freshness gate."""
        webrtc = self._webrtc
        if webrtc is None:
            self._selected_pair_cache = ""
            return
        try:
            promise = Gst.Promise.new_with_change_func(self._on_stats_reply, None)
            webrtc.emit("get-stats", None, promise)
        except Exception:
            self._selected_pair_cache = ""

    def _on_stats_reply(self, promise, _user) -> None:
        try:
            self._selected_pair_cache = self._parse_selected_pair(promise.get_reply())
        except Exception as exc:
            self._selected_pair_cache = ""
            if not getattr(self, "_stats_err_logged", False):
                self._stats_err_logged = True
                log.warning("arm: get-stats reply failed (once): %r", exc)
        # The dashboard's link readout shows the same pair (LAN / P2P / TURN).
        self._session.link.set_path(self._selected_pair_cache)

    def _parse_selected_pair(self, stats) -> str:
        """Selected ICE candidate pair from a webrtcbin `get-stats` reply, as
        "localType|remoteType|transport" (types host/srflx/prflx/relay/unknown,
        transport udp/tcp/unknown) — the station's own observation to cross-check
        the Quest-native and browser readouts (TURN Phase 1). A `relay` type is
        the marker that the arm channels are on a TURN relay. Returns "" when no
        pair is nominated yet. Carries no IP address, so it is safe to log."""
        try:
            if stats is None:
                return ""
            by_id: dict[str, Gst.Structure] = {}
            for i in range(stats.n_fields()):
                name = stats.nth_field_name(i)
                value = stats.get_value(name)
                if isinstance(value, Gst.Structure):
                    by_id[name] = value
            # A candidate-pair stat is the one referencing a local+remote id;
            # prefer the nominated pair, falling back to the first one seen.
            pair = None
            for cand in by_id.values():
                if not (cand.has_field("local-candidate-id")
                        and cand.has_field("remote-candidate-id")):
                    continue
                ok, nominated = cand.get_boolean("nominated")
                if ok and nominated:
                    pair = cand
                    break
                if pair is None:
                    pair = cand
            if pair is None:
                # Diagnostic (once): which stat structures this GStreamer emits,
                # so a schema drift (e.g. pairs nested under "transport") is
                # visible in the journal instead of a silently empty path=.
                if by_id and not getattr(self, "_stats_shape_logged", False):
                    self._stats_shape_logged = True
                    log.info("arm: get-stats has no candidate-pair stat; ids=%s",
                             sorted(by_id)[:24])
                return ""
            local = by_id.get(pair.get_string("local-candidate-id") or "")
            remote = by_id.get(pair.get_string("remote-candidate-id") or "")
            transport = "unknown"
            if local is not None and local.has_field("protocol"):
                transport = local.get_string("protocol") or "unknown"
            return f"{self._cand_type(local)}|{self._cand_type(remote)}|{transport}"
        except Exception as exc:
            if not getattr(self, "_stats_err_logged", False):
                self._stats_err_logged = True
                log.warning("arm: get-stats parse failed (once): %r", exc)
            return ""

    @staticmethod
    def _cand_type(cand) -> str:
        # GStreamer already reports the RFC ICE type strings host/srflx/prflx/
        # relay, matching the Quest-native and browser encodings.
        if cand is not None and cand.has_field("candidate-type"):
            value = cand.get_string("candidate-type")
            if value:
                return value
        return "unknown"

    def _send_ctl(self, data: bytes) -> None:
        with self._ch_lock:
            ch = self._armctl_ch
        if ch is not None and self._channel_open(ch):
            try:
                ch.emit("send-data", GLib.Bytes.new(data))
            except Exception:  # channel closing under us; drop
                pass

    @staticmethod
    def _channel_open(channel) -> bool:
        try:
            return channel.get_property("ready-state") == GstWebRTC.WebRTCDataChannelState.OPEN
        except Exception:
            return True
