"""Motion journal: the last N seconds of what the servo loop saw and did.

One row per servo tick (target, plan, commanded and actual pose/joints,
state codes, backend fields) plus event rows (engage, burst end, fault,
protective stop). The log keeps every ~10th target only; this is the record
that makes direction checks and fault forensics possible
(MIMICRY-TELEOP-PLAN.md B5).

Threading: `record`/`event` run on the servo thread and only append to a
bounded deque under a short lock. `dump` copies the rows under the lock and
writes the file outside it, from whatever thread calls it -- the servo loop
hands automatic dumps to a short-lived writer thread so file I/O never runs
on the actuator thread.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from collections import deque

log = logging.getLogger("arm_bridge.journal")


def _clean(value):
    """JSON-safe, compact: floats rounded, tuples as lists, non-finite as None."""
    if isinstance(value, float):
        return round(value, 4) if math.isfinite(value) else None
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items()}
    return value


class MotionJournal:
    def __init__(self, max_rows: int, meta: dict | None = None,
                 wall_clock=time.time) -> None:
        self._rows: deque = deque(maxlen=max(1, int(max_rows)))
        self._lock = threading.Lock()
        self._meta = dict(meta or {})
        self._wall = wall_clock
        self.enabled = max_rows > 0

    def __len__(self) -> int:
        with self._lock:
            return len(self._rows)

    def record(self, row: dict) -> None:
        if not self.enabled:
            return
        with self._lock:
            self._rows.append(row)

    def event(self, t: float, kind: str, **fields) -> None:
        self.record({"t": t, "event": kind, **fields})

    def rows(self) -> list[dict]:
        with self._lock:
            return list(self._rows)

    def dump(self, directory: str, reason: str) -> tuple[str | None, int]:
        """Write the buffer as JSONL (a meta header line, then the rows).
        Returns (path, row count); (None, 0) when disabled, empty or the
        directory cannot be written (logged, never raised)."""
        rows = self.rows()
        if not rows:
            return None, 0
        wall = self._wall()
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(wall))
        safe = "".join(c if c.isalnum() else "-" for c in reason)[:40].strip("-") or "dump"
        try:
            os.makedirs(directory, exist_ok=True)
            path = os.path.join(directory, f"journal-{stamp}-{safe}.jsonl")
            with open(path, "w", encoding="utf-8") as fh:
                header = {"journal": 1, "reason": reason, "wall_time": wall,
                          "rows": len(rows), **self._meta}
                fh.write(json.dumps(_clean(header)) + "\n")
                for row in rows:
                    fh.write(json.dumps(_clean(row), separators=(",", ":")) + "\n")
        except OSError as exc:
            log.warning("journal: cannot write %s (%s)", directory, exc)
            return None, 0
        log.info("journal: wrote %d rows to %s (%s)", len(rows), path, reason)
        return path, len(rows)

    def dump_async(self, directory: str, reason: str) -> None:
        threading.Thread(target=self.dump, args=(directory, reason),
                         name="journal-dump", daemon=True).start()
