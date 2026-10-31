#!/usr/bin/env python3
"""SpecsReach relay -- Snap Spectacles hand tracking -> the arm bridge.

The Spectacles Lens (SpecsReach `ArmRelayClient.ts`) can open a WebSocket but not a
WebRTC PeerConnection, so this relay stands between the two:

    Lens --ws:// JSON @ ~50 Hz--> specs_relay --WebRTC `arm` + `armctl`--> arm bridge :8444

To the bridge it is just another headset (like tools/arm_test_peer): it opens
the same two DataChannels, sends HELLO, echoes the STATUS motion permit and
streams the 44-byte heartbeat. The bridge is unchanged, so all of its safety
(workspace, keep-outs, IK holds, ramps, watchdog, smooth stops) still applies.

The Lens is deliberately dumb, as in SpecsGrip: it sends raw hand keypoints,
the clutch pinch and the head direction. The clutch logic, gates and the
mimicry mapping live here (`teleop_map.py`) and are tuned in
`specs_relay.json` -- no Lens rebuild to retune.

Safety on this hop:
- No Lens message for `lens_timeout_s` (specs_relay.json, 1 s: the glasses
  send in bursts) -> deadman released.
- Only the newest Lens connection is listened to; a replaced one is ignored.
- The bridge link exists only while a Lens is connected (plus a short linger),
  so the relay never holds the bridge's single peer slot for nothing.
- Power-on (ENABLE) is an operator action: keyboard `e`, never automatic
  unless `--auto-enable` is given.

    python3 specs_relay/specs_relay.py --bridge-host 127.0.0.1 --port 8766

Requires GStreamer webrtcbin + websockets: run inside the station image
(see specs_relay/README.md).
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import random
import signal
import socket
import sys
import threading
import time

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstWebRTC", "1.0")
gi.require_version("GstSdp", "1.0")
from gi.repository import GLib, Gst, GstSdp, GstWebRTC  # noqa: E402

import websockets  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teleop_map import LensSample, Teleop, Tuning, TwinSample  # noqa: E402  (also puts arm_bridge on sys.path)
from arm_bridge import protocol  # noqa: E402

LINGER_S = 10.0          # keep the bridge link this long after the Lens leaves
RETRY_S = 2.0            # bridge reconnect interval
LENS_STATE_HZ = 10.0     # relay -> Lens HUD updates


def log(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')} relay: {msg}", flush=True)


_DECODER = json.JSONDecoder()


def split_json(text: str) -> list:
    """Every JSON value in one text frame. The Spectacles WebSocket sometimes
    coalesces queued sends into one frame ('{...}{...}'); json.loads would
    reject it whole, and the dropped hand samples read as a silent Lens."""
    out, i, n = [], 0, len(text)
    while i < n:
        while i < n and text[i].isspace():
            i += 1
        if i == n:
            break
        value, i = _DECODER.raw_decode(text, i)
        out.append(value)
    return out


def lan_ips() -> list[str]:
    """Best guess at the addresses the glasses can reach (no packets sent)."""
    ips = set()
    for probe in ("192.168.1.1", "10.0.0.1", "8.8.8.8"):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect((probe, 9))
                ips.add(s.getsockname()[0])
        except OSError:
            pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


class LensLinkStats:
    """Where Lens gaps come from, logged every REPORT_S while samples flow.
    `arrival` gaps are what the relay sees; `lens` gaps come from the Lens's
    own timestamps. A big arrival gap with small lens gaps = the network held
    messages back and delivered them in a burst (often as one merged frame);
    a big lens gap = the glasses themselves stopped sending."""
    REPORT_S = 5.0

    def __init__(self) -> None:
        self._reset(0.0)
        self._last_arrival = None
        self._last_t = None

    def _reset(self, now: float) -> None:
        self.since = now
        self.msgs = self.frames = self.merged = 0
        self.max_arrival_ms = self.max_lens_ms = 0.0
        self.over_300 = 0

    def frame(self, n: int) -> None:
        self.frames += 1
        if n > 1:
            self.merged += 1

    def hand(self, now: float, lens_t) -> None:
        self.msgs += 1
        if self._last_arrival is not None and now - self._last_arrival < 5.0:
            gap = (now - self._last_arrival) * 1000.0
            self.max_arrival_ms = max(self.max_arrival_ms, gap)
            if gap > 300.0:
                self.over_300 += 1
        if isinstance(lens_t, (int, float)) and self._last_t is not None:
            dt = lens_t - self._last_t
            if 0 <= dt < 5000:
                self.max_lens_ms = max(self.max_lens_ms, dt)
        self._last_arrival = now
        self._last_t = lens_t if isinstance(lens_t, (int, float)) else None

    def report(self, now: float) -> str:
        if now - self.since < self.REPORT_S:
            return ""
        span = now - self.since
        line = ""
        if self.msgs:
            line = (f"lens link: {self.msgs / span:.0f} msg/s, max gap {self.max_arrival_ms:.0f} ms "
                    f"arriving / {self.max_lens_ms:.0f} ms sent, {self.over_300} gaps > 300 ms, "
                    f"{self.merged} merged frames")
        self._reset(now)
        return line


# ---- bridge side: one WebRTC peer, like arm_test_peer --------------------------
class BridgeLink:
    """The relay's PeerConnection to the arm bridge. Owned by the asyncio loop;
    webrtcbin calls back on its own threads, and `send_arm` / `send_ctl` may be
    called from any thread."""

    def __init__(self, host: str, port: int, path: str, on_frame, on_ready) -> None:
        self.host, self.port, self.path = host, port, path
        self._on_frame = on_frame        # (ControlFrame) on a webrtcbin thread
        self._on_ready = on_ready        # () when armctl opens and HELLO was sent
        self.state = "down"
        self.session_id = 0
        self.hello_ok = False
        self._webrtc = None
        self._pipe = None
        self._arm_ch = None
        self._ctl_ch = None
        self._mids: list[str] = []
        self._loop: asyncio.AbstractEventLoop | None = None
        self._out: asyncio.Queue | None = None
        self._failed: asyncio.Event | None = None
        self._req_lock = threading.Lock()
        self._req_id = 100

    def next_request_id(self) -> int:
        with self._req_lock:
            self._req_id = self._req_id % 0xFFFFFFF0 + 1
            return self._req_id

    @property
    def arm_open(self) -> bool:
        return self._open(self._arm_ch)

    @property
    def ctl_open(self) -> bool:
        return self._open(self._ctl_ch)

    @staticmethod
    def _open(ch) -> bool:
        try:
            return ch is not None and \
                ch.get_property("ready-state") == GstWebRTC.WebRTCDataChannelState.OPEN
        except Exception:
            return False

    def send_arm(self, data: bytes) -> bool:
        return self._send(self._arm_ch, data)

    def send_ctl(self, data: bytes) -> bool:
        return self._send(self._ctl_ch, data)

    def _send(self, ch, data: bytes) -> bool:
        if not self._open(ch):
            return False
        try:
            ch.emit("send-data", GLib.Bytes.new(data))
            return True
        except Exception:
            return False

    # ---- lifecycle --------------------------------------------------------
    async def run(self, wanted) -> None:
        """Keep a link up while `wanted()` is true; reconnect on loss."""
        self._loop = asyncio.get_running_loop()
        while True:
            if not wanted():
                await asyncio.sleep(0.5)
                continue
            try:
                await self._session(wanted)
            except (OSError, websockets.exceptions.WebSocketException, asyncio.TimeoutError) as exc:
                log(f"bridge link failed: {exc!r}")
            self.state = "down"
            await asyncio.sleep(RETRY_S)

    async def _session(self, wanted) -> None:
        url = f"ws://{self.host}:{self.port}{self.path}"
        self.state = "connecting"
        self.hello_ok = False
        self._out = asyncio.Queue()
        self._failed = asyncio.Event()
        async with websockets.connect(url, open_timeout=5, max_size=2**20) as ws:
            log(f"bridge signaling connected ({url})")
            self._build()
            try:
                self._signal({"type": "hello"})
                self._create_offer()
                sender = asyncio.create_task(self._sender(ws))
                receiver = asyncio.create_task(self._receiver(ws))
                while wanted() and not self._failed.is_set() and not receiver.done():
                    await asyncio.sleep(0.25)
                if not wanted():
                    log("no Lens connected: releasing the bridge")
                await self._out.put({"type": "bye", "v": 1})
                await asyncio.sleep(0.1)
                sender.cancel()
                receiver.cancel()
                await asyncio.gather(sender, receiver, return_exceptions=True)
            finally:
                self._teardown()

    async def _sender(self, ws) -> None:
        while True:
            msg = await self._out.get()
            await ws.send(json.dumps(msg))

    async def _receiver(self, ws) -> None:
        async for raw in ws:
            if not isinstance(raw, str):
                continue
            msg = json.loads(raw)
            kind = msg.get("type")
            if kind == "answer":
                self._apply_answer(msg.get("sdp", ""))
            elif kind == "candidate" and msg.get("candidate"):
                mid = msg.get("mid", "")
                if mid in self._mids and self._webrtc is not None:
                    self._webrtc.emit("add-ice-candidate", self._mids.index(mid), msg["candidate"])
            elif kind == "error":
                log(f"bridge signaling error: {msg}")

    def _signal(self, msg: dict) -> None:
        """Queue a signaling message from any thread."""
        msg.setdefault("v", 1)
        if self._loop is not None and self._out is not None:
            self._loop.call_soon_threadsafe(self._out.put_nowait, msg)

    def _build(self) -> None:
        pipe = Gst.Pipeline.new("specs-relay")
        webrtc = Gst.ElementFactory.make("webrtcbin", "arm")
        webrtc.set_property("bundle-policy", GstWebRTC.WebRTCBundlePolicy.MAX_BUNDLE)
        pipe.add(webrtc)
        webrtc.connect("on-ice-candidate", self._on_ice_candidate)
        webrtc.connect("notify::connection-state", self._on_conn_state)
        # webrtcbin must be live before create-data-channel, and the channels
        # must exist before the offer so it carries the m=application line.
        pipe.set_state(Gst.State.PLAYING)
        pipe.get_state(5 * Gst.SECOND)
        arm_opts = Gst.Structure.new_empty("options")
        arm_opts.set_value("ordered", False)
        arm_opts.set_value("max-retransmits", 0)
        ctl_opts = Gst.Structure.new_empty("options")
        ctl_opts.set_value("ordered", True)
        self._arm_ch = webrtc.emit("create-data-channel", "arm", arm_opts)
        self._ctl_ch = webrtc.emit("create-data-channel", "armctl", ctl_opts)
        self._ctl_ch.connect("on-open", self._on_ctl_open)
        self._ctl_ch.connect("on-message-data", self._on_ctl_data)
        for ch in (self._arm_ch, self._ctl_ch):
            ch.connect("on-close", lambda _c: self._failed and self._loop.call_soon_threadsafe(self._failed.set))
        self._pipe, self._webrtc = pipe, webrtc

    def _teardown(self) -> None:
        pipe = self._pipe
        self._pipe = self._webrtc = self._arm_ch = self._ctl_ch = None
        self._mids = []
        self.hello_ok = False
        if pipe is not None:
            pipe.set_state(Gst.State.NULL)

    def _create_offer(self) -> None:
        promise = Gst.Promise.new_with_change_func(self._on_offer_created, None)
        self._webrtc.emit("create-offer", None, promise)

    def _on_offer_created(self, promise, _user) -> None:
        reply = promise.get_reply()
        if reply is None or self._webrtc is None:
            return
        offer = reply.get_value("offer")
        sdp = offer.sdp.as_text()
        self._mids = [ln[len("a=mid:"):].strip() for ln in sdp.splitlines() if ln.startswith("a=mid:")]
        self._webrtc.emit("set-local-description", offer, Gst.Promise.new())
        self._signal({"type": "offer", "sdp": sdp})

    def _apply_answer(self, sdp_text: str) -> None:
        _res, msg = GstSdp.SDPMessage.new()
        GstSdp.sdp_message_parse_buffer(sdp_text.encode("utf-8"), msg)
        answer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, msg)
        self._webrtc.emit("set-remote-description", answer, Gst.Promise.new())

    def _on_ice_candidate(self, _el, mline, cand) -> None:
        mid = self._mids[mline] if mline < len(self._mids) else ""
        self._signal({"type": "candidate", "candidate": cand, "mid": mid})

    def _on_conn_state(self, webrtc, _pspec) -> None:
        state = webrtc.get_property("connection-state")
        if state in (GstWebRTC.WebRTCPeerConnectionState.FAILED,
                     GstWebRTC.WebRTCPeerConnectionState.CLOSED):
            log(f"bridge peerconnection {state.value_nick}")
            if self._failed is not None and self._loop is not None:
                self._loop.call_soon_threadsafe(self._failed.set)

    def _on_ctl_open(self, _ch) -> None:
        self.session_id = random.randint(1, 0xFFFFFFFE)
        self.state = "connected"
        self.send_ctl(protocol.encode_hello(1, self.session_id))
        log("bridge armctl open -> HELLO")
        self._on_ready()

    def _on_ctl_data(self, _ch, gbytes) -> None:
        try:
            frame = protocol.decode_control_frame(gbytes.get_data())
        except protocol.ProtocolError:
            return
        if frame.type == protocol.TYPE_REPLY:
            r = protocol.decode_reply(frame)
            if r.request_id == 1:
                self.hello_ok = bool(r.ok) and r.code in protocol.PROTOCOL_VERSIONS
                log(f"HELLO {'ok' if self.hello_ok else 'REFUSED'} (code {r.code}) {r.msg}")
                return
        self._on_frame(frame)


# ---- the relay -----------------------------------------------------------------
ACTIONS = {
    "e": ("enable (SET_ENABLED 1; on the AUBO: power on + brake release)", None),
    "d": ("disable (SET_ENABLED 0; on the AUBO: POWER OFF, brakes on)", None),
    "h": ("home", protocol.ARM_ACTION_HOME),
    "x": ("cancel home", protocol.ARM_ACTION_HOME_CANCEL),
    "r": ("recover / clear fault", protocol.ARM_ACTION_RECOVER),
    "t": ("retreat into the envelope", protocol.ARM_ACTION_RETREAT),
}


class Relay:
    def __init__(self, args, tuning: Tuning) -> None:
        self.args = args
        self.teleop = Teleop(tuning=tuning)
        self.lock = threading.Lock()
        self.link = BridgeLink(args.bridge_host, args.bridge_port, args.bridge_path,
                               self._on_frame, self._on_link_ready)
        self.lens = None                  # the one connected Lens websocket
        self.lens_boot = None
        self.lens_left_at = -1e9
        self.seq = 0
        self.sent = 0
        self.stop = threading.Event()
        self.cal_start = None             # first calibration point, or None
        self.last_reply = ""
        self.lens_link = LensLinkStats()

    # ---- bridge callbacks (webrtcbin threads) -----------------------------
    def _on_link_ready(self) -> None:
        if self.args.auto_enable:
            self.link.send_ctl(protocol.encode_set_enabled(2, True))
            log("--auto-enable: SET_ENABLED(1) sent")
        else:
            log("bridge ready. Press  e + Enter  to enable the arm (power on).")

    def _on_frame(self, frame) -> None:
        if frame.type == protocol.TYPE_STATUS:
            st = protocol.decode_status(frame)
            with self.lock:
                self.teleop.on_status(st, time.monotonic())
        elif frame.type == protocol.TYPE_REPLY:
            r = protocol.decode_reply(frame)
            self.last_reply = f"{'ok' if r.ok else 'REFUSED'} {r.code} {r.msg}".strip()
            log(f"reply req={r.request_id}: {self.last_reply}")

    def bridge_wanted(self) -> bool:
        return (self.args.keep_bridge or self.lens is not None
                or time.monotonic() - self.lens_left_at < LINGER_S)

    # ---- heartbeat thread ------------------------------------------------
    def heartbeat_loop(self) -> None:
        period = 1.0 / self.args.hz
        next_t = time.monotonic()
        last_note = ""
        while not self.stop.is_set():
            now = time.monotonic()
            with self.lock:
                cmd = self.teleop.tick(now)
                note = self.teleop.note
            if note != last_note:
                log(note)
                last_note = note
            if cmd is not None and self.link.hello_ok and self.link.arm_open:
                self.seq = (self.seq + 1) & 0xFFFFFFFF
                hb = protocol.Heartbeat(protocol.FLAG_DEADMAN if cmd.deadman else 0,
                                        self.link.session_id, self.seq, *cmd.pose,
                                        cmd.gripper, cmd.permit)
                if self.link.send_arm(protocol.encode_heartbeat(hb)):
                    self.sent += 1
            next_t += period
            delay = next_t - time.monotonic()
            if delay < -period:          # fell behind (GC, load): resync, don't burst
                next_t = time.monotonic()
                delay = 0.0
            self.stop.wait(max(delay, 0.0))

    # ---- Lens side (asyncio) ----------------------------------------------
    async def lens_handler(self, ws, _path=None) -> None:
        peer = getattr(ws, "remote_address", None)
        if self.stop.is_set():
            await ws.close(code=1001, reason="relay stopping")
            return
        old, self.lens = self.lens, ws
        if old is not None:
            # Usually the same glasses reconnecting after a Wi-Fi drop: the old
            # socket is dead, and awaiting its close handshake would stall this
            # one for the full close timeout. Switch now, close it aside.
            log("a second Lens connected: replacing the old connection")
            asyncio.create_task(old.close(code=1012, reason="replaced"))
        log(f"Lens connected from {peer}")
        try:
            async for raw in ws:
                if self.lens is not ws:
                    # Replaced by a newer connection. Its close handshake can
                    # take seconds (the glasses answer it late), and messages
                    # still arriving here would interleave two hand streams.
                    break
                if not isinstance(raw, str):
                    continue
                try:
                    msgs = split_json(raw)
                    self.lens_link.frame(len(msgs))
                    for msg in msgs:
                        self._on_lens_message(msg)
                except (ValueError, KeyError, TypeError) as exc:
                    log(f"bad Lens message dropped: {exc}")
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            if self.lens is ws:
                self.lens = None
                self.lens_left_at = time.monotonic()
                with self.lock:
                    self.teleop.forget_lens()
                log(f"Lens disconnected ({peer})")

    def _on_lens_message(self, msg: dict) -> None:
        kind = msg.get("type")
        if kind == "hand":
            sample = LensSample.from_json(msg)
            self.lens_link.hand(time.monotonic(), msg.get("t"))
            with self.lock:
                self.teleop.on_lens(sample, time.monotonic())
        elif kind == "twin":
            # Digital twin mode: the model's TCP target, already in the arm frame.
            sample = TwinSample.from_json(msg)
            self.lens_link.hand(time.monotonic(), msg.get("t"))
            with self.lock:
                self.teleop.on_twin(sample, time.monotonic())
        elif kind == "hello":
            boot = msg.get("boot")
            if boot != self.lens_boot and self.teleop.room_forward is not None:
                # A new Lens launch has a new world origin: the old calibration
                # would steer the arm the wrong way.
                with self.lock:
                    self.teleop.room_forward = None
                    self.teleop.tuning.mode = "view"
                log("new Lens session: room calibration cleared (back to view mode)")
            self.lens_boot = boot
            log(f"Lens hello from {msg.get('device', 'unknown device')}: "
                f"drive={msg.get('drive')} clutch={msg.get('clutch')}")

    async def lens_state_loop(self) -> None:
        while True:
            report = self.lens_link.report(time.monotonic())
            if report:
                log(report)
            ws = self.lens
            if ws is not None:
                try:
                    await ws.send(json.dumps(self.state_dict()))
                except websockets.exceptions.ConnectionClosed:
                    pass
            await asyncio.sleep(1.0 / LENS_STATE_HZ)

    def state_dict(self) -> dict:
        with self.lock:
            st = self.teleop.status
            fresh = st is not None and time.monotonic() - self.teleop._status_t < 0.5
            bridge = self.link.state
            if bridge == "connected" and not self.link.hello_ok:
                bridge = "handshake"
            return {
                "type": "state",
                "bridge": bridge,
                "arm": st.arm_state.name if fresh else "NO STATUS",
                "err": st.error_code if fresh else 0,
                "warn": st.warn_code if fresh else 0,
                "hold": bool(st.hold_latched) if fresh else False,
                "engaged": self.teleop.engaged,
                "mode": self.teleop.tuning.mode,
                "calibrating": self.cal_start is not None,
                "note": self.teleop.note,
                "reply": self.last_reply,
                # The arm's actual TCP (mm, RPY deg; null without a fresh STATUS),
                # so the Lens's digital twin can mirror the real arm.
                "pose": ([round(v, 2) for v in (st.x, st.y, st.z, st.roll, st.pitch, st.yaw)]
                         if fresh else None),
            }

    # ---- keyboard --------------------------------------------------------
    def keyboard_loop(self) -> None:
        self.print_help()
        for line in sys.stdin:
            cmd = line.strip().lower()
            if not cmd:
                continue
            if cmd == "q":
                self.stop.set()
                return
            self.command(cmd)

    def print_help(self) -> None:
        print("\nkeys (type + Enter):")
        for k, (label, _a) in ACTIONS.items():
            print(f"  {k}  {label}")
        print("  c  calibrate room frame (press twice: hand over the arm base, then along arm +X)")
        print("  v  view mode (no calibration; arm +X = where you face, view_yaw_deg)")
        print("  s  status      ?  help      q  quit\n", flush=True)

    def command(self, cmd: str) -> None:
        if cmd in ACTIONS:
            if not self.link.hello_ok:
                log("bridge not connected yet")
                return
            if self.teleop.engaged:
                log("release the pinch first")
                return
            rid = self.link.next_request_id()
            if cmd == "e":
                self.link.send_ctl(protocol.encode_set_enabled(rid, True))
            elif cmd == "d":
                self.link.send_ctl(protocol.encode_set_enabled(rid, False))
            else:
                self.link.send_ctl(protocol.encode_arm_action(rid, ACTIONS[cmd][1]))
            log(f"sent: {ACTIONS[cmd][0]} (req {rid})")
        elif cmd == "c":
            self.calibrate_step()
        elif cmd == "v":
            with self.lock:
                self.teleop.tuning.mode = "view"
            log(f"view mode (view_yaw_deg {self.teleop.tuning.view_yaw_deg:g})")
        elif cmd == "s":
            log(json.dumps(self.state_dict()) + f" heartbeats_sent={self.sent}")
        else:
            self.print_help()

    def calibrate_step(self) -> None:
        with self.lock:
            wrist = self.teleop.latest_wrist(time.monotonic())
            engaged = self.teleop.engaged
        if engaged:
            log("calibration: release the pinch first")
            return
        if wrist is None:
            log("calibration: the drive hand is not tracked -- hold it in view")
            return
        if self.cal_start is None:
            self.cal_start = wrist
            log("calibration 1/2 recorded. Move the hand >= 20 cm along the arm's +X, press c again")
            return
        start, self.cal_start = self.cal_start, None
        with self.lock:
            ok = self.teleop.calibrate_from_points(start, wrist)
        log("calibration done: room mode" if ok else
            "calibration: move too short (< 20 cm horizontally) -- press c to start again")


def load_tuning(path: str) -> Tuning:
    if not path or not os.path.exists(path):
        return Tuning()
    with open(path, encoding="utf-8") as fh:
        return Tuning.from_dict(json.load(fh))


async def amain(relay: Relay, args) -> None:
    server = await websockets.serve(relay.lens_handler, args.listen, args.port,
                                    max_size=2**16, ping_interval=5, ping_timeout=5)
    for ip in lan_ips() or ["<this-host-ip>"]:
        log(f"Lens URL: ws://{ip}:{args.port}")
    # Ctrl-C / docker stop take the same path as `q`: release the bridge
    # cleanly first (bye -> the bridge holds the arm), then shut down.
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, relay.stop.set)
    tasks = [asyncio.create_task(relay.link.run(relay.bridge_wanted)),
             asyncio.create_task(relay.lens_state_loop())]
    while not relay.stop.is_set():
        await asyncio.sleep(0.2)
    log("stopping: releasing the bridge")
    server.close()                     # no new Lens (it would re-open the bridge)
    lens, relay.lens = relay.lens, None
    relay.lens_left_at = -1e9          # bridge_wanted() -> False: the link says bye
    if lens is not None:
        await lens.close(code=1001, reason="relay stopping")
    await asyncio.sleep(0.6)
    for t in tasks:
        t.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


def main() -> None:
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(description="Spectacles hand tracking -> arm bridge relay.")
    p.add_argument("--bridge-host", default="127.0.0.1")
    p.add_argument("--bridge-port", type=int, default=8444)
    p.add_argument("--bridge-path", default="/arm")
    p.add_argument("--listen", default="0.0.0.0", help="address for the Lens WebSocket")
    p.add_argument("--port", type=int, default=8766, help="port for the Lens WebSocket")
    p.add_argument("--config", default=os.path.join(here, "specs_relay.json"))
    p.add_argument("--hz", type=float, default=50.0, help="heartbeat rate (bridge expects ~50)")
    p.add_argument("--auto-enable", action="store_true",
                   help="send SET_ENABLED(1) as soon as the bridge link is up")
    p.add_argument("--keep-bridge", action="store_true",
                   help="hold the bridge link even with no Lens connected")
    args = p.parse_args()
    if not 20.0 <= args.hz <= 100.0:
        p.error("--hz must be within 20..100")
    try:
        tuning = load_tuning(args.config)
    except (ValueError, OSError) as exc:
        p.error(f"config {args.config}: {exc}")

    Gst.init(None)
    relay = Relay(args, tuning)
    log(f"mode={tuning.mode} view_yaw_deg={tuning.view_yaw_deg:g} "
        f"scale={tuning.position_scale:g} bridge=ws://{args.bridge_host}:{args.bridge_port}{args.bridge_path}")
    threading.Thread(target=relay.heartbeat_loop, name="heartbeat", daemon=True).start()
    if sys.stdin.isatty():
        threading.Thread(target=relay.keyboard_loop, name="keyboard", daemon=True).start()
    else:
        log("stdin is not a terminal: keyboard commands off (run docker with -it)")
    try:
        asyncio.run(amain(relay, args))
    except KeyboardInterrupt:
        pass
    relay.stop.set()
    log("stopped")
    # GStreamer's native threads can keep the interpreter from exiting; the
    # bridge link is already closed and the logs are flushed, so leave now.
    os._exit(0)


if __name__ == "__main__":
    main()
