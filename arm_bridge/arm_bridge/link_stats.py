"""Teleop link quality for the dashboard: what the station measures about the
headset link, plus what the headset reports about itself (CLIENT_STATS).

Display and alerting only -- nothing here gates motion. The watchdog and the
motion permit (session.py) hold the arm; this module explains how healthy the
path feeding them is, so the operator sees a bad link before it bites.

Producers take this monitor's lock only long enough to append:
- Session.on_heartbeat (transport thread, after it released the session lock):
  one call per in-session heartbeat -- arrival gap, skipped sequence numbers,
  the echoed motion permit, whether its target passed the workspace check.
- Session.plan (servo thread): a comm/quality hold latching.
- the transport: CLIENT_STATS reports and the selected ICE pair.
Consumer: StateHub.publish (status thread, 30 Hz) via snapshot(), which copies
the samples under the lock, computes outside it, and recomputes at most every
SNAPSHOT_PERIOD_S (the cached dict is returned in between).

Loop RTT: every STATUS carries motionPermitId = the server's monotonic clock in
ms, and the Quest echoes the newest one in each heartbeat. The first heartbeat
carrying a new permit therefore times STATUS -> Quest -> heartbeat on ONE clock
(no clock sync): the network round trip plus the Quest's turnaround -- it sends
heartbeats on a 20 ms schedule from its frame loop -- so it reads ~10-30 ms above
a ping. That is the round trip a command really experiences, hence "loop RTT".

Loss: the `arm` channel is unreliable (max-retransmits 0), so every skipped
heartbeat sequence number is a packet the network lost (or reordered behind a
newer one, which the servo loop drops all the same).
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque

from . import rotation
from .config import LinkQualityConfig
from .protocol import CAMERA_STATES, CANDIDATE_TYPES, ClientStats

_SEQ_MOD = 1 << 32
_SEQ_HALF = 1 << 31

SNAPSHOT_PERIOD_S = 0.25     # recompute cadence; the status thread asks at 30 Hz
HISTORY_BUCKET_S = 1.0       # one sparkline point per second ...
HISTORY_POINTS = 60          # ... for the last minute
CLIENT_STALE_S = 3.0         # the Quest reports at ~1 Hz; older = not shown
SILENT_AFTER_MS = 1000.0     # no heartbeat this long = the headset is silent
VIDEO_STALL_MS = 500.0       # the Quest's own video motion gate holds at 500 ms
PERMIT_RTT_MAX_MS = 10_000   # anything older is not a round trip (bogus permit)

GRADES = ("good", "fair", "poor")
THERMAL_STATES = {0: "none", 1: "light", 2: "moderate", 3: "severe",
                  4: "critical", 5: "emergency", 6: "shutdown"}
_STREAMING = 5               # CAMERA_STATES code for "streaming"


def _num(value: float | None, digits: int = 1) -> float | None:
    """JSON-safe number: NaN / inf -> None (the client's "unknown")."""
    if value is None or not math.isfinite(value):
        return None
    return round(float(value), digits)


def _pct(sorted_values: list[float], q: float) -> float:
    """Nearest-rank percentile of an ascending list (non-empty)."""
    idx = min(len(sorted_values) - 1, max(0, math.ceil(q * len(sorted_values)) - 1))
    return sorted_values[idx]


def parse_path(pair: str) -> dict | None:
    """"localType|remoteType|transport" (transport_webrtc._parse_selected_pair)
    -> {local, remote, transport, relay, source}. `local` is always the
    station's candidate. A relay on either side means the path runs through
    TURN; anything else is peer-to-peer."""
    if not pair:
        return None
    parts = (pair.split("|") + ["", "", ""])[:3]
    local, remote, transport = (p or "unknown" for p in parts)
    return {"local": local, "remote": remote, "transport": transport,
            "relay": "relay" in (local, remote), "source": "station"}


def _headset_path(headset_side: int, station_side: int) -> dict | None:
    """A selected pair as the headset reports it, turned around so `local` is
    the station's candidate like parse_path. None when the headset has none."""
    station, headset = CANDIDATE_TYPES.get(station_side), CANDIDATE_TYPES.get(headset_side)
    if not station and not headset:
        return None
    station, headset = station or "unknown", headset or "unknown"
    return {"local": station, "remote": headset, "transport": None,
            "relay": "relay" in (station, headset), "source": "headset"}


def follow_error(target, pose) -> dict | None:
    """How far the actual pose trails the commanded target: translation (mm)
    and geodesic rotation (deg). None without both poses."""
    if not target or not pose or len(target) < 6 or len(pose) < 6:
        return None
    try:
        mm = math.dist(target[:3], pose[:3])
        deg = rotation.angle_between(tuple(target[3:6]), tuple(pose[3:6]))
    except (TypeError, ValueError):
        return None
    return {"mm": _num(mm), "deg": _num(deg)}


class LinkMonitor:
    """Rolling-window link quality for the current arm session."""

    def __init__(self, config: LinkQualityConfig | None = None,
                 clock=time.monotonic) -> None:
        self._cfg = config if config is not None else LinkQualityConfig()
        self._clock = clock
        self._lock = threading.Lock()      # producers <-> snapshot copy
        self._compute_lock = threading.Lock()  # snapshot's own state (cache, hysteresis)
        self._path = ""
        self._reset_locked(None)
        # Consumer-owned (under _compute_lock).
        self._cache: dict | None = None
        self._cache_at = -math.inf
        self._cache_gen = -1
        self._poor_since: float | None = None
        self._ok_since: float | None = None
        self._degraded = False
        self._bucket_next = -math.inf
        self._history: deque = deque(maxlen=HISTORY_POINTS)   # loop RTT p50 per second

    # ---- producers ---------------------------------------------------------
    def _reset_locked(self, now: float | None) -> None:
        self._since = now                  # session start; None = no session
        self._gen = getattr(self, "_gen", 0) + 1
        self._hb: deque = deque()          # (t, gap_ms | None, skipped)
        self._rtt: deque = deque()         # (t, rtt_ms)
        self._last_permit = 0
        self._heartbeats = 0
        self._lost = 0
        self._target_rejects = 0
        self._latches = {"watchdog": 0, "permit": 0}
        self._client: ClientStats | None = None
        self._client_at = -math.inf
        self._video: dict[int, deque] = {}  # slot -> (t, received, lost)

    def on_session_start(self, now: float | None = None) -> None:
        """HELLO: a new session starts with empty windows and counters."""
        now = self._clock() if now is None else now
        with self._lock:
            self._reset_locked(now)

    def on_session_end(self) -> None:
        with self._lock:
            self._reset_locked(None)
            self._path = ""

    def _horizon(self, now: float) -> float:
        return now - max(self._cfg.window_s, HISTORY_BUCKET_S)

    def on_heartbeat(self, now: float, gap_ms: float | None, skipped: int,
                     permit: int, target_ok: bool = True) -> None:
        """One in-session heartbeat (fresh seq; target valid or not)."""
        with self._lock:
            if self._since is None:
                return
            self._hb.append((now, gap_ms, skipped))
            self._heartbeats += 1
            self._lost += skipped
            if not target_ok:
                self._target_rejects += 1
            permit &= 0xFFFFFFFF
            if permit:
                newer = (self._last_permit == 0
                         or 0 < (permit - self._last_permit) % _SEQ_MOD < _SEQ_HALF)
                if newer:
                    self._last_permit = permit
                    rtt = ((int(now * 1000.0) & 0xFFFFFFFF) - permit) % _SEQ_MOD
                    if rtt <= PERMIT_RTT_MAX_MS:
                        self._rtt.append((now, float(rtt)))
            horizon = self._horizon(now)
            while self._hb and self._hb[0][0] < horizon:
                self._hb.popleft()
            while self._rtt and self._rtt[0][0] < horizon:
                self._rtt.popleft()

    def on_hold_latched(self, reason: str) -> None:
        """A comm/quality hold latched (`watchdog` or `permit`)."""
        with self._lock:
            if self._since is not None and reason in self._latches:
                self._latches[reason] += 1

    def on_client_stats(self, stats: ClientStats, now: float | None = None) -> None:
        """The headset's own report (CLIENT_STATS, ~1 Hz)."""
        now = self._clock() if now is None else now
        with self._lock:
            if self._since is None:
                return
            self._client, self._client_at = stats, now
            horizon = now - self._cfg.window_s - 1.0
            for cam in stats.cameras:
                hist = self._video.setdefault(cam.slot, deque())
                if hist and (cam.packets_received < hist[-1][1]
                             or cam.packets_lost < hist[-1][2]):
                    hist.clear()            # the stream restarted: counters reset
                hist.append((now, cam.packets_received, cam.packets_lost))
                while hist and hist[0][0] < horizon:
                    hist.popleft()

    def set_path(self, pair: str) -> None:
        """The arm PeerConnection's selected ICE pair, from webrtcbin stats."""
        with self._lock:
            self._path = pair or ""

    # ---- consumer ----------------------------------------------------------
    def snapshot(self, now: float | None = None) -> dict:
        """The dashboard's `link` block. Cheap: cached for SNAPSHOT_PERIOD_S."""
        now = self._clock() if now is None else now
        with self._compute_lock:
            with self._lock:
                gen = self._gen
                fresh = (self._cache is not None and gen == self._cache_gen
                         and now - self._cache_at < SNAPSHOT_PERIOD_S)
                if fresh:
                    return self._cache
                since = self._since
                hb = list(self._hb)
                rtt = list(self._rtt)
                totals = {"heartbeats": self._heartbeats, "lost": self._lost,
                          "target_rejects": self._target_rejects,
                          "watchdog_latches": self._latches["watchdog"],
                          "permit_latches": self._latches["permit"]}
                client, client_at = self._client, self._client_at
                video = {slot: list(h) for slot, h in self._video.items()}
                path = self._path
            if gen != self._cache_gen:      # a new session: fresh hysteresis + history
                self._poor_since = self._ok_since = None
                self._degraded = False
                self._history.clear()
                self._bucket_next = -math.inf
            snap = self._compute(now, since, hb, rtt, totals, client, client_at,
                                 video, path)
            self._cache, self._cache_at, self._cache_gen = snap, now, gen
            return snap

    def _compute(self, now, since, hb, rtt, totals, client, client_at,
                 video, path) -> dict:
        cfg = self._cfg
        if since is None:
            self._poor_since = self._ok_since = None
            self._degraded = False
            return {"active": False, "path": parse_path(path)}

        window = self._window_stats(now, since, hb, rtt, cfg.window_s)
        last_hb = hb[-1][0] if hb else since
        open_gap_ms = (now - last_hb) * 1000.0
        silent = open_gap_ms >= SILENT_AFTER_MS
        client_json = None
        if client is not None and now - client_at <= CLIENT_STALE_S:
            client_json = self._client_json(client, now - client_at, video, now)

        grade, reasons = (None, [])
        if not silent:
            grade, reasons = self._grade(window, client_json)
        self._update_alert(now, grade == "poor")

        if now >= self._bucket_next:
            last = self._window_stats(now, since, hb, rtt, HISTORY_BUCKET_S)
            self._history.append(last["rtt"]["p50_ms"] if last["rtt"] else None)
            # A fixed 1 s cadence (snapshot timing jitter must not skip
            # seconds); restart it after a gap longer than a bucket.
            self._bucket_next += HISTORY_BUCKET_S
            if now >= self._bucket_next:
                self._bucket_next = now + HISTORY_BUCKET_S

        # The station's own view of the arm pair when webrtcbin reports one
        # (1.24 does not, for a DataChannel-only PeerConnection), else the
        # headset's.
        arm_path = parse_path(path)
        if arm_path is None and client is not None and now - client_at <= CLIENT_STALE_S:
            arm_path = _headset_path(client.arm_local_candidate, client.arm_remote_candidate)

        return {
            "active": True,
            "window_s": cfg.window_s,
            "age_s": _num(now - since),
            "rtt": window["rtt"],
            "loss_pct": window["loss_pct"],
            "hb_hz": window["hb_hz"],
            "gap_p95_ms": window["gap_p95_ms"],
            "gap_max_ms": _num(max(window["gap_max_ms"] or 0.0, open_gap_ms), 0),
            "silent_ms": _num(open_gap_ms, 0) if silent else None,
            "path": arm_path,
            "grade": grade,
            "reasons": reasons,
            "degraded": self._degraded,
            "session": totals,
            # The last minute of loop RTT, one median per second (None = a
            # second without a sample): the menu's sparkline, full on open.
            "history_rtt_ms": list(self._history),
            "client": client_json,
        }

    @staticmethod
    def _window_stats(now, since, hb, rtt, span_s) -> dict:
        t0 = now - span_s
        hbs = [h for h in hb if h[0] > t0]
        rtts = sorted(r for t, r in rtt if t > t0)
        received = len(hbs)
        skipped = sum(h[2] for h in hbs)
        total = received + skipped
        span = min(span_s, now - since)
        gaps = sorted(g for _, g, _ in hbs if g is not None)
        return {
            "rtt": ({"p50_ms": _num(_pct(rtts, 0.5)), "min_ms": _num(rtts[0]),
                     "p95_ms": _num(_pct(rtts, 0.95)), "n": len(rtts)}
                    if rtts else None),
            "loss_pct": _num(100.0 * skipped / total, 2) if total else None,
            "hb_hz": _num(received / span) if span >= 0.5 else None,
            "gap_p95_ms": _num(_pct(gaps, 0.95), 0) if gaps else None,
            "gap_max_ms": _num(gaps[-1], 0) if gaps else None,
        }

    def _grade(self, window: dict, client: dict | None) -> tuple[str | None, list[str]]:
        cfg = self._cfg
        level = -1
        reasons: list[tuple[int, str]] = []

        def rate(value, fair, poor, text):
            nonlocal level
            if value is None:
                return
            lvl = 2 if value >= poor else 1 if value >= fair else 0
            level = max(level, lvl)
            if lvl:
                reasons.append((lvl, text))

        rtt = window["rtt"]
        if rtt:
            rate(rtt["p50_ms"], cfg.rtt_fair_ms, cfg.rtt_poor_ms,
                 f"loop RTT {rtt['p50_ms']:.0f} ms")
        loss = window["loss_pct"]
        rate(loss, cfg.loss_fair_pct, cfg.loss_poor_pct,
             f"heartbeat loss {loss:.1f} %" if loss is not None else "")
        gap = window["gap_max_ms"]
        rate(gap, cfg.gap_fair_ms, cfg.gap_poor_ms,
             f"heartbeat gap {gap:.0f} ms" if gap is not None else "")
        for cam in (client or {}).get("cameras", []):
            if cam["state"] != "streaming":
                continue
            age = cam["last_packet_age_ms"]
            if age is not None and age >= VIDEO_STALL_MS:
                level = 2
                reasons.append((2, f"cam{cam['slot']} video stalled {age:.0f} ms"))
            vloss = cam["loss_pct"]
            rate(vloss, cfg.loss_fair_pct, cfg.loss_poor_pct,
                 f"cam{cam['slot']} video loss {vloss:.1f} %" if vloss is not None else "")
        if level < 0:
            return None, []
        reasons.sort(key=lambda r: -r[0])
        return GRADES[level], [text for _, text in reasons]

    def _update_alert(self, now: float, poor: bool) -> None:
        """Hysteresis for the alert lane: poor for alert_after_s raises it,
        not-poor for clear_after_s clears it -- a single spike never flickers it."""
        if poor:
            self._ok_since = None
            if self._poor_since is None:
                self._poor_since = now
            if not self._degraded and now - self._poor_since >= self._cfg.alert_after_s:
                self._degraded = True
        else:
            self._poor_since = None
            if self._ok_since is None:
                self._ok_since = now
            if self._degraded and now - self._ok_since >= self._cfg.clear_after_s:
                self._degraded = False

    def _client_json(self, c: ClientStats, age_s: float, video: dict, now: float) -> dict:
        wifi = None
        if c.wifi_rssi_dbm or c.wifi_link_mbps or c.wifi_freq_mhz:
            wifi = {"rssi_dbm": c.wifi_rssi_dbm or None,
                    "link_mbps": c.wifi_link_mbps or None,
                    "freq_mhz": c.wifi_freq_mhz or None}
        cameras = []
        for cam in c.cameras:
            loss = None
            hist = [h for h in video.get(cam.slot, []) if h[0] >= now - self._cfg.window_s - 1.0]
            if len(hist) >= 2:
                d_rx = hist[-1][1] - hist[0][1]
                d_lost = hist[-1][2] - hist[0][2]
                if d_rx + d_lost > 0:
                    loss = _num(100.0 * d_lost / (d_rx + d_lost), 2)
            cameras.append({
                "slot": cam.slot,
                "state": CAMERA_STATES.get(cam.state, f"state {cam.state}"),
                "fps": _num(cam.fps),
                "loss_pct": loss,
                "packets_lost": cam.packets_lost,
                "last_packet_age_ms": _num(cam.last_packet_age_ms, 0),
                "decode_p50_ms": _num(cam.decode_p50_ms),
                "decode_p95_ms": _num(cam.decode_p95_ms),
                "idr_requests": cam.idr_requests,
                "decoder_resets": cam.decoder_resets,
                "incomplete_au_drops": cam.incomplete_au_drops,
            })
        return {
            "age_s": _num(age_s),
            "arm_ping_rtt_ms": _num(c.arm_ping_rtt_ms),
            "arm_transport_rtt_ms": _num(c.arm_transport_rtt_ms),
            "video_transport_rtt_ms": _num(c.video_transport_rtt_ms),
            "app_fps": _num(c.app_fps),
            "arm_path_changes": c.arm_path_changes,
            "arm_ice_reconnects": c.arm_ice_reconnects,
            "status_stale_count": c.status_stale_count,
            "video_path": _headset_path(c.video_local_candidate, c.video_remote_candidate),
            "battery_pct": c.battery_pct if c.battery_pct <= 100 else None,
            "thermal": THERMAL_STATES.get(c.thermal_status),
            "wifi": wifi,
            "cameras": cameras,
        }
