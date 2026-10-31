"""Controller programs for the AUBO driver (`AuboController` mixin).

A program is Lua text sent to the ARCS script port (`ScriptClient`, TCP 30002,
terminated by a blank line `\\r\\n\\r\\n`); the controller runs it and the
bridge steers and watches it over the driver's RPC connections
(RuntimeMachine). All calls here run on the servo thread (ProgramSupervisor),
on the driver's COMMAND connection; the state poller adds the runtime state
(and, when enabled, the executing line and trace output) to its cache.

Verified on this arm (aubo_probe `script`, `--speed-fraction`, Ctrl-C):
sendString, subscribeScriptError, getStatus, setSpeedFraction, abort,
stopJoint/stopLine. Everything else is behind `aubo.program_features` and
stays off until verified on the ARCS simulator VM, then the arm:
  line         RuntimeMachine.getPlanContext (the text is instrumented with
               setPlanContext by lua_check.instrument)
  pause, step  RuntimeMachine.pause / resume / step
  breakpoints  RuntimeMachine.setBreakPoint / clearBreakPoints
  simulate     RobotManage.setSim (the arm should stay still)
  output       Trace.peek -- textmsg() lines
"""

from __future__ import annotations

import logging
from collections import deque

from . import error_codes as ec
from .arm_interface import ArmFault
from .lua_check import normalize_newlines
from .program_runtime import IDLE, ProgramEvent, ProgramPoll

log = logging.getLogger("arm_bridge.aubo")

# ARCS RuntimeState -> bridge program state.
RUNTIME_STATES = {"Running": "running", "Retracting": "running", "Stepping": "running",
                  "Pausing": "running", "Paused": "paused", "Stopping": "stopping",
                  "Aborting": "stopping", "Stopped": "idle"}
TRACE_PEEK = 20


def _name(value) -> str:
    return str(getattr(value, "name", value))


class AuboProgramMixin:
    supports_programs = True

    def _init_programs(self) -> None:
        self._script = None                     # persistent ScriptClient
        self._prog_events: deque[ProgramEvent] = deque(maxlen=500)
        self._prog_owned = False                # a run of ours is on the controller
        self._prog_restore_speed: float | None = None
        self._prog_sim = False
        self._prog_bp_lines: tuple[int, ...] = ()
        self._trace_cmd = None                  # Trace on the command connection
        self._trace_last = 0
        # poller cache (under self._lock, like the driver's other extras)
        self._runtime_state = "Stopped"
        self._plan_line: int | None = None

    # ---- features ---------------------------------------------------------
    def program_features(self) -> frozenset[str]:
        return frozenset(self._t.program_features)

    def _feature(self, name: str) -> bool:
        return name in self._t.program_features

    # ---- poller side (called from the driver's _read_extras) ---------------
    def _program_extras(self, runtime, trace) -> dict:
        """Runtime state, plan line and trace lines for the cache. Runs on
        the poller (or the servo thread for a forced refresh, trace=None)."""
        state = _name(runtime.getStatus())
        line = None
        if state != "Stopped" and self._feature("line"):
            context = runtime.getPlanContext(-1)
            try:
                line = int(context[1]) or None
            except (TypeError, ValueError, IndexError):
                line = None
        if trace is not None and self._feature("output") and (
                state != "Stopped" or self._prog_owned):
            for msg in trace.peek(TRACE_PEEK, self._trace_last):
                stamp = int(getattr(msg, "timestamp", 0) or 0)
                if stamp <= self._trace_last:
                    continue
                self._trace_last = stamp
                args = getattr(msg, "args", None) or []
                text = " ".join(str(a) for a in args) or f"code {getattr(msg, 'code', '?')}"
                level = _name(getattr(msg, "level", ""))
                self._prog_events.append(ProgramEvent(
                    "output", text if level.lower() in ("", "info") else f"[{level}] {text}"))
        return {"runtime": state, "plan_line": line}

    # ---- script channel -----------------------------------------------------
    def _on_script_error(self, message) -> None:
        # SDK thread: only a deque append (thread-safe, never blocks).
        self._prog_events.append(ProgramEvent("error", str(message)))

    def _script_client(self):
        if self._script is not None:
            return self._script
        t = self._t
        client = self._invoke("ScriptClient", self._sdk.ScriptClient)
        rc = self._invoke("script connect", client.connect, t.host, int(t.script_port))
        if int(rc) != 0:
            raise ArmFault(f"AUBO script port {t.host}:{t.script_port} refused: rc "
                           f"{self._rc_str(rc)}", ec.CONTROLLER_UNREACHABLE)
        self._call("script login", client.login, t.user, t.password)
        self._invoke("subscribeScriptError", client.subscribeScriptError,
                     self._on_script_error)
        self._script = client
        return client

    def _close_script(self) -> None:
        client, self._script = self._script, None
        if client is not None:
            for op in ("logout", "disconnect"):
                try:
                    getattr(client, op)()
                except Exception:  # noqa: BLE001 -- teardown is best effort
                    pass

    # ---- commands (servo thread) --------------------------------------------
    def program_start(self, source: str, *, simulate: bool, speed_fraction: float,
                      breakpoints: tuple[int, ...] = (),
                      line_plan: tuple[int, ...] = ()) -> None:
        self._require_running("run a program")
        s = self._cached()
        if s["program"]:
            raise ArmFault("a controller program is already running", ec.PROGRAM_RUNNING)
        if s["freedrive"]:
            raise ArmFault("free-drive is on -- leave it before running a program",
                           ec.NOT_READY)
        if simulate and not self._feature("simulate"):
            raise ArmFault("simulated runs are not enabled (aubo.program_features)",
                           ec.UNSUPPORTED)
        self.disable_servo()                    # flush any queued teleop points
        try:
            if simulate:
                self._call("setSim(True)", self._manage.setSim, True)
                self._prog_sim = True
                if not bool(self._invoke("isSimulationEnabled",
                                         self._manage.isSimulationEnabled)):
                    raise ArmFault("the controller did not enter simulation mode", ec.TIMEOUT)
            previous = float(self._invoke("getSpeedFraction", self._motion.getSpeedFraction))
            self._call("setSpeedFraction", self._motion.setSpeedFraction, float(speed_fraction))
            self._prog_restore_speed = previous
            if breakpoints and self._feature("breakpoints"):
                self._set_breakpoints(breakpoints)
            if self._feature("output") and self._robot_if is not None:
                self._trace_cmd = self._trace_cmd or self._invoke(
                    "getTrace", self._robot_if.getTrace)
                history = self._invoke("trace peek", self._trace_cmd.peek, 50, 0) or []
                self._trace_last = max([int(getattr(m, "timestamp", 0) or 0)
                                        for m in history] + [self._trace_last])
            client = self._script_client()
            text = normalize_newlines(source).rstrip("\n") + "\r\n\r\n"
            try:
                rc = int(client.sendString(text))
            except Exception as exc:  # noqa: BLE001 -- the SDK boundary
                self._close_script()            # reconnect next time
                raise ArmFault(f"sending the program raised {type(exc).__name__}: {exc}",
                               ec.SDK_CALL_FAILED) from exc
            if rc != 0:
                raise ArmFault(f"the controller refused the program: rc {self._rc_str(rc)}",
                               ec.SDK_CALL_FAILED)
            self._prog_owned = True
            log.info("aubo: program sent (%d lines, %s, speed fraction %.2f)",
                     source.count("\n") + 1, "simulated" if simulate else "real arm",
                     speed_fraction)
        except ArmFault:
            self.program_finish()               # undo what was changed
            raise

    def _set_breakpoints(self, lines) -> None:
        self._call("clearBreakPoints", self._runtime.clearBreakPoints)
        for line in lines:
            self._call("setBreakPoint", self._runtime.setBreakPoint, int(line))
        self._prog_bp_lines = tuple(lines)

    def program_control(self, op: str) -> None:
        feature = "step" if op == "step" else "pause"
        if not self._feature(feature):
            raise ArmFault(f"{op} is not enabled (aubo.program_features)", ec.UNSUPPORTED)
        if op not in ("pause", "resume", "step"):
            raise ArmFault(f"unknown program command {op}", ec.UNSUPPORTED)
        self._require_connected()
        self._call(f"RuntimeMachine.{op}", getattr(self._runtime, op))

    def program_set_breakpoints(self, lines: tuple[int, ...]) -> None:
        if not self._feature("breakpoints"):
            raise ArmFault("breakpoints are not enabled (aubo.program_features)",
                           ec.UNSUPPORTED)
        self._require_connected()
        self._set_breakpoints(lines)

    def program_stop(self) -> None:
        """Abort the controller's program (ours or the pendant's) AND stop
        motion: AUBO's docs disagree on whether abort alone stops the arm."""
        if self._runtime is None or not self._rpc_alive():
            raise ArmFault("AUBO controller not connected", ec.CONTROLLER_UNREACHABLE)
        t = self._t
        for op, fn, args in (("abort", self._runtime.abort, ()),
                             ("stopJoint", self._motion.stopJoint, (self._stop_acc(),)),
                             ("stopLine", self._motion.stopLine,
                              (float(t.stop_line_acc), float(t.stop_line_acc_rot)))):
            try:
                self._call(op, fn, *args)
            except ArmFault as exc:
                log.warning("aubo: program stop: %s failed: %s", op, exc.message)
        self._clear_path_pending = True       # the stops hold the trajectory path
        log.warning("aubo: program stop sent (abort + stopJoint + stopLine)")

    def program_poll(self) -> ProgramPoll:
        with self._lock:
            runtime, line = self._runtime_state, self._plan_line
        state = RUNTIME_STATES.get(runtime, "running")
        if state == "idle":
            return IDLE
        reason = None
        if state == "paused":
            reason = "breakpoint" if line is not None and line in self._prog_bp_lines else "paused"
        return ProgramPoll(state, line, external=not self._prog_owned, paused_reason=reason)

    def program_events(self) -> list[ProgramEvent]:
        out = []
        while self._prog_events:
            try:
                out.append(self._prog_events.popleft())
            except IndexError:
                break
        return out

    def program_finish(self) -> None:
        if self._motion is not None and self._prog_restore_speed is not None:
            self._best("setSpeedFraction(restore)", self._motion.setSpeedFraction,
                       self._prog_restore_speed)
        if self._prog_sim and self._manage is not None:
            self._best("setSim(False)", self._manage.setSim, False)
        if self._prog_bp_lines and self._runtime is not None:
            self._best("clearBreakPoints", self._runtime.clearBreakPoints)
        self._prog_restore_speed = None
        self._prog_sim = False
        self._prog_bp_lines = ()
        self._prog_owned = False
