"""Structured in-process log bus for the dashboard (SIM-DASHBOARD-PLAN §4).

All three station log sources -- `system` (bridge lifecycle/servo), `client`
(Quest traffic), `robot` (SDK/controller boundary) -- publish LogEvents through
one bus; the dashboard WebSocket replays the ring buffer to new subscribers and
streams batches after that. `record` is the rosbag recording supervisor;
`program` is the controller-program runner (program_supervisor.py: run
lifecycle, controller errors, textmsg() output).

Design constraints (from the plan review):
- publish() is called from the servo/transport/status threads: it must be
  cheap, never block, and never raise into the caller.
- Subscriber queues are bounded, drop-OLDEST on overflow, and count their
  drops -- a slow browser can never back-pressure a bridge thread.
- The stdlib `logging` bridge exists because the bridge's journal lines are
  ordinary `logging` calls (post print-migration); mirroring them costs one
  handler, no call sites.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass, field

SOURCES = ("system", "client", "robot", "record", "program")
LEVELS = ("debug", "info", "warn", "error")

RING_CAPACITY = 10_000
SUBSCRIBER_QUEUE_CAPACITY = 2_000


@dataclass(frozen=True)
class LogEvent:
    seq: int
    ts_ms: float                    # wall clock, epoch ms (display/export)
    source: str                     # one of SOURCES
    level: str                      # one of LEVELS
    text: str
    data: dict | None = None

    def as_dict(self) -> dict:
        out = {"seq": self.seq, "ts_ms": self.ts_ms, "source": self.source,
               "level": self.level, "text": self.text}
        if self.data is not None:
            out["data"] = self.data
        return out


@dataclass
class Subscriber:
    """One dashboard connection's bounded feed. `dropped` counts events lost to
    overflow since the last drain; the server reports it so truncation is
    visible, never silent."""
    events: "queue.Queue[LogEvent]" = field(
        default_factory=lambda: queue.Queue(maxsize=SUBSCRIBER_QUEUE_CAPACITY))
    dropped: int = 0

    def drain(self, max_events: int = 500) -> tuple[list[LogEvent], int]:
        """Non-blocking: return (events, dropped_since_last_drain)."""
        out: list[LogEvent] = []
        while len(out) < max_events:
            try:
                out.append(self.events.get_nowait())
            except queue.Empty:
                break
        dropped, self.dropped = self.dropped, 0
        return out, dropped


class LogBus:
    def __init__(self, ring_capacity: int = RING_CAPACITY,
                 clock=time.time) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._seq = 0
        self._ring: deque[LogEvent] = deque(maxlen=ring_capacity)
        self._subscribers: list[Subscriber] = []

    def publish(self, source: str, level: str, text: str,
                data: dict | None = None) -> None:
        """Thread-safe, non-blocking, never raises into the caller."""
        if source not in SOURCES:
            source = "system"
        if level not in LEVELS:
            level = "info"
        with self._lock:
            self._seq += 1
            event = LogEvent(seq=self._seq, ts_ms=self._clock() * 1000.0,
                             source=source, level=level, text=text, data=data)
            self._ring.append(event)
            subscribers = list(self._subscribers)
        for sub in subscribers:
            try:
                sub.events.put_nowait(event)
            except queue.Full:
                # Drop-oldest: make room for the new event, count the loss.
                try:
                    sub.events.get_nowait()
                    sub.dropped += 1
                    sub.events.put_nowait(event)
                except (queue.Empty, queue.Full):
                    sub.dropped += 1

    def subscribe(self) -> tuple[Subscriber, list[LogEvent]]:
        """Register a subscriber; returns it plus the ring-buffer snapshot to
        replay (events published after this call land in the queue)."""
        sub = Subscriber()
        with self._lock:
            snapshot = list(self._ring)
            self._subscribers.append(sub)
        return sub, snapshot

    def unsubscribe(self, sub: Subscriber) -> None:
        with self._lock:
            try:
                self._subscribers.remove(sub)
            except ValueError:
                pass

    @property
    def subscriber_count(self) -> int:
        with self._lock:
            return len(self._subscribers)


class BusLogHandler(logging.Handler):
    """Mirror stdlib logging records into the bus.

    Source resolution: a logger named `arm_bridge.client.*` / `arm_bridge.robot.*`
    / `arm_bridge.record.*` maps to that source; everything else is `system`.
    Publishing must never raise into a logging call site (handleError contract).
    """

    _LEVELS = {logging.DEBUG: "debug", logging.INFO: "info",
               logging.WARNING: "warn", logging.ERROR: "error",
               logging.CRITICAL: "error"}

    def __init__(self, bus: LogBus) -> None:
        super().__init__()
        self._bus = bus

    def emit(self, record: logging.LogRecord) -> None:
        try:
            source = "system"
            parts = record.name.split(".")
            if len(parts) >= 2 and parts[1] in SOURCES:
                source = parts[1]
            level = self._LEVELS.get(record.levelno, "info")
            self._bus.publish(source, level, record.getMessage())
        except Exception:
            self.handleError(record)


def wire_logging(bus: LogBus, stream=None) -> logging.Logger:
    """Set up the bridge's journal: `arm_bridge.*` loggers emit bare lines to
    stdout (preserving the pre-migration `print` journal format for docker logs)
    and mirror into the bus. Idempotent -- safe to call once from main().
    """
    root = logging.getLogger("arm_bridge")
    root.setLevel(logging.INFO)
    root.propagate = False
    if not any(isinstance(h, BusLogHandler) for h in root.handlers):
        stream_handler = logging.StreamHandler(stream)
        stream_handler.setFormatter(logging.Formatter("%(message)s"))
        root.addHandler(stream_handler)
        root.addHandler(BusLogHandler(bus))
    return root
