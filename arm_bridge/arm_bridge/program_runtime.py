"""Program-runtime types shared by the arm backends and the supervisor.

A program (Lua) runs ON the arm controller; the bridge only starts, steers
and watches it. Backends that can run programs set
`ArmController.supports_programs` and implement the `program_*` methods
(arm_interface.py); `ProgramSupervisor` (program_supervisor.py) drives them
from the servo thread.

`FakeLuaRuntime` is the mock backend's controller stand-in: it walks the
program's statement lines on a timer -- no Lua is executed -- and honours
pause / step / breakpoints, `textmsg("...")` output and an injectable error
(`-- @mock-error <message>` on a line), so the whole Programs tab can be built
and tested without a robot.
"""

from __future__ import annotations

import re
import threading
import time
from collections import deque
from dataclasses import dataclass

# Optional features a backend may support once VERIFIED on its controller.
# Run and Stop are always available when supports_programs is set.
#   line         live executing line (instrumented setPlanContext)
#   pause, step  RuntimeMachine pause/resume and single step
#   breakpoints  stop at gutter breakpoints
#   simulate     dry run in the controller's simulation mode (arm stays still)
#   output       textmsg() lines from the controller's trace log
PROGRAM_FEATURES = ("line", "pause", "step", "breakpoints", "simulate", "output")


@dataclass(frozen=True)
class ProgramPoll:
    state: str                       # idle | running | paused | stopping
    line: int | None = None          # executing line, when known
    external: bool = False           # running, but not started by the bridge (pendant)
    paused_reason: str | None = None  # "breakpoint" | "step" | "paused"


IDLE = ProgramPoll("idle")


@dataclass(frozen=True)
class ProgramEvent:
    kind: str                        # output | error
    text: str
    line: int | None = None


_TEXTMSG_RE = re.compile(r"""textmsg\(\s*(["'])(.*?)\1""")
_MOCK_ERROR_RE = re.compile(r"--\s*@mock-error\b\s*(.*)$")


class FakeLuaRuntime:
    """Line-walking controller stand-in (see the module doc). Thread-safe:
    the mock backend calls it from the servo thread, tests from anywhere."""

    BASE_DWELL_S = 0.4               # per line at speed fraction 1.0

    def __init__(self, clock=time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._state = "idle"
        self._plan: list[int] = []
        self._idx = -1
        self._next_at = 0.0
        self._dwell = self.BASE_DWELL_S
        self._breakpoints: frozenset[int] = frozenset()
        self._outputs: dict[int, str] = {}
        self._errors: dict[int, str] = {}
        self._events: deque[ProgramEvent] = deque(maxlen=500)
        self._paused_reason: str | None = None
        self.started = 0                 # test aid

    def start(self, source: str, line_plan, speed_fraction: float,
              breakpoints=()) -> None:
        lines = source.split("\n")
        with self._lock:
            if self._state != "idle":
                raise RuntimeError("a program is already running")
            self._plan = list(line_plan)
            self._outputs, self._errors = {}, {}
            for number in self._plan:
                text = lines[number - 1] if 0 < number <= len(lines) else ""
                out = _TEXTMSG_RE.search(text)
                if out:
                    self._outputs[number] = out.group(2)
                err = _MOCK_ERROR_RE.search(text)
                if err:
                    self._errors[number] = err.group(1) or "mock error"
            self._breakpoints = frozenset(breakpoints)
            self._dwell = min(3.0, self.BASE_DWELL_S / max(float(speed_fraction), 0.1))
            self._idx = -1
            self._state = "running"
            self._paused_reason = None
            self.started += 1
            self._advance_locked(self._clock())

    def _advance_locked(self, now: float) -> None:
        self._idx += 1
        if self._idx >= len(self._plan):
            self._state = "idle"
            return
        line = self._plan[self._idx]
        if line in self._outputs:
            self._events.append(ProgramEvent("output", self._outputs[line], line))
        if line in self._errors:
            self._events.append(ProgramEvent(
                "error", f'[string "program"]:{line}: {self._errors[line]}', line))
            self._state = "idle"
            return
        if line in self._breakpoints and self._state == "running":
            self._state = "paused"
            self._paused_reason = "breakpoint"
        self._next_at = now + self._dwell

    def poll(self) -> ProgramPoll:
        now = self._clock()
        with self._lock:
            if self._state == "running" and now >= self._next_at:
                self._advance_locked(now)
            if self._state == "idle":
                return IDLE
            line = self._plan[self._idx] if 0 <= self._idx < len(self._plan) else None
            return ProgramPoll(self._state, line,
                               paused_reason=self._paused_reason if self._state == "paused" else None)

    def control(self, op: str) -> None:
        with self._lock:
            if op == "pause" and self._state == "running":
                self._state, self._paused_reason = "paused", "paused"
            elif op == "resume" and self._state == "paused":
                self._state, self._paused_reason = "running", None
                self._next_at = self._clock() + self._dwell
            elif op == "step" and self._state == "paused":
                self._advance_locked(self._clock())
                if self._state != "idle":
                    self._state, self._paused_reason = "paused", "step"
            else:
                raise RuntimeError(f"cannot {op} while {self._state}")

    def set_breakpoints(self, lines) -> None:
        with self._lock:
            self._breakpoints = frozenset(lines)

    def stop(self) -> None:
        with self._lock:
            self._state = "idle"

    def events(self) -> list[ProgramEvent]:
        with self._lock:
            out = list(self._events)
            self._events.clear()
            return out
