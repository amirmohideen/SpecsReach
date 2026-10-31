"""Program supervisor: the run lifecycle of a controller program.

The dashboard prepares a run on its own thread (check, params, line
instrumentation) and submits it here; everything that touches the arm happens
in `service()`, which the servo loop calls every tick -- so the bridge keeps
its single-actuator rule even though the motion comes from the controller.

State machine of the active run:

    starting -> running <-> paused -> (stopping) -> finished | stopped | error

`starting` lasts until the controller reports the program (or an error
arrives); a program shorter than one status poll ends as finished after
START_GRACE_S. A run the controller aborts on its own (pendant Stop) ends as
`stopped` with `stop_source: "controller"`, never as `finished`. A program the
bridge did not start (the pendant's) shows as `external`; Stop aborts it too.

Watchdog (operator decision 2026-10-11): while a run is active and no
dashboard page has been connected for `programs.watchdog_s`, the run is
stopped -- someone must be watching while the arm moves.

`snapshot()` is the dashboard's view (a dict swapped atomically, GIL-safe to
read from any thread). Finished runs go to `on_run_end(record)`, which the
dashboard's programs store writes to disk off this thread.
"""

from __future__ import annotations

import logging
import re
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime

from . import error_codes
from .arm_interface import ArmFault
from .program_runtime import IDLE, ProgramPoll
from .protocol import ArmState

log = logging.getLogger("arm_bridge.program")

START_GRACE_S = 1.5
STOP_RETRY_S = 1.0
STOP_TIMEOUT_S = 5.0
ACTIVE = ("starting", "running", "paused", "stopping")
_LUA_LINE_RE = re.compile(r":(\d+):")


@dataclass
class RunRequest:
    path: str                        # programs-relative file path
    source: str                      # as written, params applied (the editor's text)
    sent: str                        # what the controller receives (instrumented or = source)
    params: dict = field(default_factory=dict)
    speed_fraction: float = 0.25
    simulate: bool = False
    breakpoints: tuple[int, ...] = ()
    line_plan: tuple[int, ...] = ()
    started_by: str = ""
    warnings: list = field(default_factory=list)


class PendingResult:
    """A queued command's outcome; the dashboard thread waits on it."""

    def __init__(self) -> None:
        self._event = threading.Event()
        self.ok = False
        self.msg = ""
        self.data: dict = {}

    def fulfill(self, ok: bool, msg: str = "", data: dict | None = None) -> None:
        self.ok, self.msg, self.data = ok, msg, data or {}
        self._event.set()

    def wait(self, timeout: float) -> "PendingResult":
        if not self._event.wait(timeout):
            self.ok, self.msg = False, "the servo loop did not answer in time"
        return self


class ViewerCounter:
    """Dashboard pages connected right now (WebSocket `?role=ui`); the run
    watchdog reads it. Tools on /ws (e.g. the arm-state recorder) don't count."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._n = 0

    def add(self, delta: int) -> None:
        with self._lock:
            self._n = max(0, self._n + delta)

    def get(self) -> int:
        return self._n


class ProgramSupervisor:
    def __init__(self, arm, watchdog_s: float = 5.0, clock=time.monotonic,
                 wall_clock=time.time, viewer_count=None, on_run_end=None) -> None:
        self._arm = arm
        self._watchdog_s = float(watchdog_s)
        self._clock = clock
        self._wall_clock = wall_clock
        self._viewer_count = viewer_count
        self._on_run_end = on_run_end
        self._lock = threading.Lock()
        self._commands: deque[tuple[str, object, PendingResult]] = deque()
        # servo-thread state
        self._run: dict | None = None
        self._last: dict | None = None
        self._poll: ProgramPoll = IDLE
        self._breakpoints: tuple[int, ...] = ()
        self._no_viewer_since: float | None = None
        self._run_counter = 0
        self._snapshot: dict = self._build_snapshot(clock())

    # ---- any thread --------------------------------------------------------
    def submit(self, op: str, payload=None) -> PendingResult:
        pending = PendingResult()
        with self._lock:
            self._commands.append((op, payload, pending))
        return pending

    def snapshot(self) -> dict:
        return self._snapshot

    @property
    def busy(self) -> bool:
        """A run of ours is active, or the controller runs someone else's."""
        return self._run is not None or self._poll.state != "idle"

    @property
    def features(self) -> frozenset[str]:
        try:
            return frozenset(self._arm.program_features())
        except Exception:  # noqa: BLE001 -- informational
            return frozenset()

    # ---- servo thread ------------------------------------------------------
    def service(self, now: float, program_mode: bool, arm_moving: bool = False) -> None:
        """`arm_moving`: the bridge itself is moving the arm (HOME / back-out);
        a run is refused until it ends."""
        with self._lock:
            commands = list(self._commands)
            self._commands.clear()
        for op, payload, pending in commands:
            try:
                if op == "run" and arm_moving:
                    ok, msg, data = False, "the arm is moving home -- wait for it or press STOP", {}
                else:
                    ok, msg, data = self._handle(op, payload, now, program_mode)
            except ArmFault as exc:
                ok, msg, data = False, exc.message, {"code": exc.code}
            pending.fulfill(ok, msg, data)
        try:
            self._poll = self._arm.program_poll()
        except Exception as exc:  # noqa: BLE001 -- poll is contractually non-raising
            log.debug("program poll failed: %r", exc)
        for event in self._safe_events():
            self._on_event(event)
        if self._run is not None:
            self._check_arm_fault()
            self._advance(now)
        self._watchdog(now)
        self._snapshot = self._build_snapshot(now)

    def stop_now(self, now: float, source: str) -> str:
        """Abort whatever program runs (ours or external) and stop motion."""
        run = self._run
        if run is None and self._poll.state == "idle":
            return "no program running"
        failed = None
        try:
            self._arm.program_stop()
        except ArmFault as exc:
            failed = exc.message
            log.error("program: stop failed: %s", exc.message)
        if run is None:
            if failed:
                return f"could not stop the controller's program: {failed}"
            log.warning("program: stopped the controller's program (%s)", source)
            return "stopped the controller's program"
        if run["state"] != "stopping":
            run["state"] = "stopping"
            run["stop_source"] = source
            run["stop_at"] = run["stop_retry_at"] = now
        return f"stopping {run['path']}"

    # ---- internals ---------------------------------------------------------
    def _safe_events(self):
        try:
            return self._arm.program_events()
        except Exception as exc:  # noqa: BLE001
            log.debug("program events failed: %r", exc)
            return []

    def _handle(self, op: str, payload, now: float, program_mode: bool):
        features = self.features
        run = self._run
        if op == "run":
            req: RunRequest = payload
            if not program_mode:
                return False, "switch the station to Program mode first", {}
            if run is not None:
                return False, f"{run['path']} is still running", {}
            if self._poll.state != "idle":
                return False, "the controller is already running a program (pendant?)", {}
            if req.simulate and "simulate" not in features:
                return False, "simulated runs are not available on this controller", {}
            self._arm.program_start(
                req.sent, simulate=req.simulate, speed_fraction=req.speed_fraction,
                breakpoints=req.breakpoints if "breakpoints" in features else (),
                line_plan=req.line_plan)
            self._breakpoints = tuple(req.breakpoints)
            self._run = self._new_run(req, now)
            log.info("program: run %s started: %s (%s, speed %d%%%s)",
                     self._run["run_id"], req.path,
                     "simulated" if req.simulate else "real arm",
                     round(req.speed_fraction * 100),
                     "" if not req.params else ", " + " ".join(
                         f"{k}={v:g}" for k, v in req.params.items()))
            if "output" not in features and "textmsg" in req.sent:
                log.info("program: its textmsg() lines will not show here -- 'output' "
                         "is not enabled for this controller")
            return True, f"started {req.path}", {"run_id": self._run["run_id"]}
        if op in ("pause", "resume", "step"):
            if run is None or run["state"] not in ("running", "paused"):
                return False, "no program is running", {}
            if op == "step" and run["state"] != "paused":
                return False, "pause the program before stepping", {}
            needed = "step" if op == "step" else "pause"
            if needed not in features:
                return False, f"{op} is not available on this controller", {}
            self._arm.program_control(op)
            return True, op, {}
        if op == "stop":
            return True, self.stop_now(now, str(payload or "dashboard")), {}
        if op == "breakpoints":
            self._breakpoints = tuple(sorted({int(v) for v in payload or ()}))
            if run is not None and "breakpoints" in features:
                self._arm.program_set_breakpoints(self._breakpoints)
            return True, "breakpoints set", {"breakpoints": list(self._breakpoints)}
        return False, f"unknown program command '{op}'", {}

    def _new_run(self, req: RunRequest, now: float) -> dict:
        self._run_counter += 1
        wall = self._wall_clock()
        stem = re.sub(r"[^A-Za-z0-9_-]+", "_", req.path.rsplit("/", 1)[-1].removesuffix(".lua"))
        run_id = f"{datetime.fromtimestamp(wall):%Y%m%d-%H%M%S}-{self._run_counter:03d}-{stem}"
        return {"run_id": run_id, "path": req.path, "request": req, "state": "starting",
                "started_mono": now, "started_ms": wall * 1000.0, "seen": False,
                "line": None, "paused_reason": None, "error": None,
                "stop_source": None, "stop_at": None, "stop_retry_at": None,
                "max_line": None, "stop_warned": False}

    def _on_event(self, event) -> None:
        if event.kind == "output":
            log.info("%s", event.text)
            return
        line = event.line
        if line is None:
            m = _LUA_LINE_RE.search(event.text)
            line = int(m.group(1)) if m else None
        log.error("program error%s: %s", f" (line {line})" if line else "", event.text)
        run = self._run
        if run is not None and run["error"] is None:
            run["error"] = {"line": line, "msg": event.text}

    def _check_arm_fault(self) -> None:
        """A protective stop / fault during a run is the run's error. The
        controller only PAUSES the program then; Clear fault aborts it first."""
        run = self._run
        if run["error"] is not None:
            return
        try:
            snap = self._arm.read_snapshot()        # cached on the real backends
        except Exception:  # noqa: BLE001
            return
        if not (snap.protective_stop or snap.arm_state in (ArmState.ERROR, ArmState.PROTECTIVE_STOP)):
            return
        code = snap.error_code
        kind = "protective stop" if snap.protective_stop else "arm fault"
        label = error_codes.error_label(code) if code else kind
        run["error"] = {"line": run["line"], "msg": f"{kind}: {label} (code {code})"}
        log.error("program: %s during run %s%s: %s (code %s) -- the controller paused the "
                  "program; Clear fault aborts it", kind, run["run_id"],
                  f" at line {run['line']}" if run["line"] else "", label, code)

    def _advance(self, now: float) -> None:
        run, poll = self._run, self._poll
        if poll.line is not None:
            run["line"] = poll.line
        state = run["state"]
        if state == "starting":
            if poll.state in ("running", "paused"):
                run["state"], run["seen"] = poll.state, True
                run["paused_reason"] = poll.paused_reason
            elif run["error"] is not None:
                self._end(now, "error")
            elif poll.state == "stopping":
                if run["stop_source"] is None:          # aborted before we saw it run
                    run["stop_source"] = "controller"
            elif poll.state == "idle" and now - run["started_mono"] >= START_GRACE_S:
                self._end(now, "stopped" if run["stop_source"] else "finished")
        elif state in ("running", "paused"):
            if poll.state == "idle":
                if run["error"]:
                    self._end(now, "error")
                else:
                    # Went through Aborting/Stopping without a Stop of ours:
                    # the pendant (or the controller) ended it. Not "finished".
                    self._end(now, "stopped" if run["stop_source"] else "finished")
            elif poll.state == "stopping":
                if run["stop_source"] is None:
                    run["stop_source"] = "controller"
                    log.warning("program: run %s is being aborted by the controller / pendant",
                                run["run_id"])
            else:
                run["state"] = "paused" if poll.state == "paused" else "running"
                run["paused_reason"] = poll.paused_reason
        elif state == "stopping":
            if poll.state == "idle":
                self._end(now, "error" if run["error"] else "stopped")
            elif now - run["stop_retry_at"] >= STOP_RETRY_S:
                run["stop_retry_at"] = now
                if now - run["stop_at"] >= STOP_TIMEOUT_S and not run["stop_warned"]:
                    run["stop_warned"] = True
                    log.error("program: still running %.0f s after Stop -- use the pendant "
                              "e-stop if the arm is moving", now - run["stop_at"])
                try:
                    self._arm.program_stop()
                except ArmFault as exc:
                    log.error("program: stop retry failed: %s", exc.message)

    def _end(self, now: float, result: str) -> None:
        run = self._run
        self._run = None
        try:
            self._arm.program_finish()
        except Exception as exc:  # noqa: BLE001 -- restore is best effort
            log.warning("program: restoring controller settings failed: %r", exc)
        req: RunRequest = run["request"]
        duration = now - run["started_mono"]
        summary = {"run_id": run["run_id"], "path": run["path"], "result": result,
                   "started_ms": run["started_ms"],
                   "ended_ms": run["started_ms"] + duration * 1000.0,
                   "duration_s": round(duration, 2), "error": run["error"],
                   "stop_source": run["stop_source"], "simulate": req.simulate,
                   "speed_fraction": req.speed_fraction, "line": run["line"]}
        self._last = summary
        if result == "error":
            err = run["error"] or {}
            log.error("program: run %s ended with an error%s: %s", run["run_id"],
                      f" at line {err['line']}" if err.get("line") else "", err.get("msg", "?"))
        elif result == "stopped":
            log.warning("program: run %s stopped by %s after %.1f s", run["run_id"],
                        run["stop_source"], duration)
        else:
            log.info("program: run %s finished in %.1f s", run["run_id"], duration)
        if self._on_run_end is not None:
            record = dict(summary, params=req.params, started_by=req.started_by,
                          warnings=req.warnings, breakpoints=list(req.breakpoints),
                          source=req.source, sent=req.sent)
            try:
                self._on_run_end(record)
            except Exception as exc:  # noqa: BLE001 -- history must not break the loop
                log.warning("program: recording run history failed: %r", exc)

    def _watchdog(self, now: float) -> None:
        run = self._run
        active = run is not None and run["state"] in ("starting", "running", "paused")
        if not active or self._watchdog_s <= 0 or self._viewer_count is None:
            self._no_viewer_since = None
            return
        try:
            viewers = int(self._viewer_count())
        except Exception:  # noqa: BLE001
            viewers = 1
        if viewers > 0:
            self._no_viewer_since = None
            return
        if self._no_viewer_since is None:
            self._no_viewer_since = now
        elif now - self._no_viewer_since >= self._watchdog_s:
            log.warning("program: no dashboard has been watching for %.0f s -- stopping %s",
                        self._watchdog_s, run["path"])
            self.stop_now(now, "watchdog (nobody watching)")
            self._no_viewer_since = None

    def _build_snapshot(self, now: float) -> dict:
        run, poll = self._run, self._poll
        snap = {"features": sorted(self.features), "breakpoints": list(self._breakpoints),
                "last": self._last, "external": False}
        if run is not None:
            req: RunRequest = run["request"]
            snap.update(state=run["state"], run_id=run["run_id"], path=run["path"],
                        line=run["line"], paused_reason=run["paused_reason"],
                        started_ms=run["started_ms"],
                        elapsed_s=round(now - run["started_mono"], 1),
                        simulate=req.simulate, speed_fraction=req.speed_fraction,
                        error=run["error"], stop_source=run["stop_source"])
        elif poll.state != "idle":
            snap.update(state=poll.state, run_id=None, path=None, line=poll.line,
                        paused_reason=poll.paused_reason, external=True)
        else:
            snap.update(state="idle", run_id=None, path=None, line=None)
        return snap
