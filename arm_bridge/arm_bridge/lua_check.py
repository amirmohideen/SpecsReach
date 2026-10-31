"""Lua program checker for the dashboard's Programs tab (AUBO ARCS scripts).

One implementation serves three jobs, so the editor, the Run gate and the
controller-bound text can never disagree:

  * check(): a permissive Lua 5.3 parser (5.4 `<attrib>`s accepted) that
    reports the FIRST syntax error like `luac` does, plus the AUBO rules:
      - structure: the chunk ends with `return function(api) ... end`
        (AUBO's documented TCP-script form) and calls `select_robot(...)` --
        without it the robot API is bare C++ class tables, and calling those
        crashed the controller software once (aubo-control-notes.md §10);
      - operator-only calls (DENIED) -- errors, also when aliased
        (`local p = poweroff`), reached through `_G.x` / `_G['x']` or named
        in a string;
      - unknown functions (not Lua stdlib, not the vendored AUBO API list, not
        defined in the file), unknown module members (`sched.sleeep`,
        `string.formatt`), `setSpeedFraction`, `enterCritical`, `print`,
        TCP/payload changes and the escape hatches the checker cannot see
        through (`load`, `dofile`, `os.execute`, `io.*`, `_G[...]`) --
        warnings the Run dialog makes the operator acknowledge;
      - `break` outside a loop / `goto` without a label -- errors, as luac.
    It also reads the `@param` lines (form fields).
  * apply_params(): rewrites only the value of those `local NAME = <num>`
    lines, so every line keeps its number.
  * instrument(): adds, on the SAME line, `setPlanContext(<tid>, <line>, "")`
    before each statement after `select_robot` and a `sched.cancel_point()`
    at the top of each loop body -- what pendant-generated code does -- so the
    controller reports the executing line (RuntimeMachine.getPlanContext) and
    tight loops stay abortable. Line numbers are unchanged. The anchor must be
    a main-body `local _ENV = require('aubo').sched.select_robot(1)`: the
    inserted calls are bare names and resolve only through that `_ENV`.

The rules catch mistakes; they are not a sandbox (the PIN, the servo-thread
supervisor and the controller are the gates). The controller remains the
final judge of syntax. Pure Python, stdlib only.
"""

from __future__ import annotations

import difflib
import json
import logging
import math
import re
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

API_PATH = Path(__file__).parent / "data" / "aubo_lua_api.json"

log = logging.getLogger("arm_bridge.program.check")

KEYWORDS = frozenset(
    "and break do else elseif end false for function goto if in local nil not "
    "or repeat return then true until while".split())

# Lua 5.3 standard globals, plus the logging helpers ARCS defines even in the
# bare environment (aubo-control-notes.md §10).
LUA_GLOBALS = frozenset(
    "assert collectgarbage dofile error getmetatable ipairs load loadfile next "
    "pairs pcall print rawequal rawget rawlen rawset require select setmetatable "
    "tonumber tostring type xpcall _G _VERSION _ENV coroutine debug io math os "
    "package string table utf8 log_info log_warn log_error log_debug".split())

# Operator-only calls: never from a program. Reason shown in the editor.
DENIED = {
    "setUnlockProtectiveStop": "clearing a protective stop is an operator action",
    "poweroff": "power is switched by the operator (dashboard / pendant)",
    "poweron": "power is switched by the operator (dashboard / pendant)",
    "startup": "brake release is an operator action",
    "releaseRobotBrake": "brake release is an operator action",
    "lockRobotBrake": "brakes are an operator action",
    "setSim": "the bridge owns simulation mode (the Simulate switch)",
    "setOperationalMode": "the operational mode is set on the pendant",
    "setLinkModeEnable": "link mode is set on the pendant",
    "setPersistentParameters": "it writes persistent controller settings",
    "setRobotZero": "it changes the joint zero calibration",
}
# Allowed, but the Run dialog makes the operator acknowledge them.
_ESCAPE = ("escape-hatch",
           "runs code or touches the controller's filesystem that this checker cannot see")
CAUTION = {
    "setSpeedFraction": ("speed-override",
                         "overrides the speed chosen in the Run dialog for the rest of the run"),
    "enterCritical": ("delayed-stop", "defers Stop by up to 5 s while in the critical section"),
    "setTcpOffset": ("commissioning", "changes the commissioned TCP offset until the bridge reconnects"),
    "setPayload": ("commissioning", "changes the commissioned payload until the bridge reconnects"),
    # Escape hatches: the only way a program can do something the editor does
    # not show. Global calls here; `os.x` / `io.x` members are in CAUTION_MEMBERS.
    "load": _ESCAPE, "loadstring": _ESCAPE, "loadfile": _ESCAPE, "dofile": _ESCAPE,
    "rawget": _ESCAPE, "rawset": _ESCAPE,
}
CAUTION_MEMBERS = {("os", "execute"): _ESCAPE, ("os", "exit"): _ESCAPE,
                   ("os", "remove"): _ESCAPE, ("os", "rename"): _ESCAPE,
                   ("os", "tmpname"): _ESCAPE, ("io", "*"): _ESCAPE}

# Lua 5.3 standard library members, for the module-member check (a misspelt
# `string.formatt` is a runtime error on the controller, not here).
LUA_STD_MEMBERS = {
    "string": frozenset("byte char dump find format gmatch gsub len lower match pack "
                        "packsize rep reverse sub unpack upper".split()),
    "table": frozenset("concat insert move pack remove sort unpack".split()),
    "math": frozenset("abs acos asin atan ceil cos deg exp floor fmod huge log max "
                      "maxinteger min mininteger modf pi rad random randomseed sin sqrt "
                      "tan tointeger type ult".split()),
    "os": frozenset("clock date difftime execute exit getenv remove rename setlocale "
                    "time tmpname".split()),
    "io": frozenset("close flush input lines open output popen read stderr stdin stdout "
                    "tmpfile type write".split()),
    "coroutine": frozenset("close create isyieldable resume running status wrap yield".split()),
    "utf8": frozenset("char charpattern codes codepoint len offset".split()),
}

# Instrumentation text (see the module doc). One local is added right after
# the select_robot statement; the names cannot clash with user code.
_TP_SCHED = "__tp_sched"
_ANCHOR_TEXT = f"; local {_TP_SCHED} = require('aubo').sched;"
_LINE_TEXT = ';setPlanContext(' + _TP_SCHED + '.current_thread_id(), {line}, ""); '
_CANCEL_TEXT = f" {_TP_SCHED}.cancel_point();"

MAX_SOURCE_BYTES = 256 * 1024


class LuaSyntaxError(Exception):
    def __init__(self, line: int, col: int, message: str) -> None:
        super().__init__(f"{line}:{col}: {message}")
        self.line, self.col, self.message = line, col, message


# ---- tokens -----------------------------------------------------------------

@dataclass(frozen=True)
class Token:
    kind: str        # name | keyword | number | string | op | eof
    value: str
    line: int
    col: int         # 1-based
    pos: int         # offset of the first char
    end: int         # offset after the last char

    def near(self) -> str:
        if self.kind == "eof":
            return "'<eof>'"
        return f"'{self.value}'" if len(self.value) <= 40 else f"'{self.value[:37]}...'"


_OPS3 = ("...",)
_OPS2 = ("..", "==", "~=", "<=", ">=", "<<", ">>", "//", "::")
_OPS1 = frozenset("+-*/%^#&~|<>=(){}[];:,.")
_DEC_RE = re.compile(r"(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?")
_HEX_RE = re.compile(r"0[xX](?:[0-9a-fA-F]+\.?[0-9a-fA-F]*|\.[0-9a-fA-F]+)(?:[pP][+-]?\d+)?")


def _is_name_start(c: str) -> bool:
    return c == "_" or ("a" <= c <= "z") or ("A" <= c <= "Z")


def _is_name_char(c: str) -> bool:
    return _is_name_start(c) or ("0" <= c <= "9")


class _Lexer:
    def __init__(self, src: str) -> None:
        self.src = src
        self.i = 0
        self.line = 1
        self.line_start = 0

    def _col(self, pos: int) -> int:
        return pos - self.line_start + 1

    def _err(self, message: str, pos: int | None = None) -> LuaSyntaxError:
        pos = self.i if pos is None else pos
        return LuaSyntaxError(self.line, self._col(pos), message)

    def _newline(self, pos_after: int) -> None:
        self.line += 1
        self.line_start = pos_after

    def _long_level(self, pos: int) -> int | None:
        """At '[': the level of a long bracket `[==[`, or None."""
        src = self.src
        if pos >= len(src) or src[pos] != "[":
            return None
        j = pos + 1
        while j < len(src) and src[j] == "=":
            j += 1
        return j - pos - 1 if j < len(src) and src[j] == "[" else None

    def _skip_long(self, pos: int, level: int, what: str) -> int:
        """Skip a long bracket starting at `pos` ('['); returns the end offset."""
        close = "]" + "=" * level + "]"
        start_line = self.line
        k = self.src.find(close, pos)
        if k < 0:
            raise LuaSyntaxError(start_line, self._col(pos), f"unfinished long {what}")
        segment = self.src[pos:k]
        newlines = segment.count("\n")
        if newlines:
            self.line += newlines
            self.line_start = pos + segment.rfind("\n") + 1
        return k + len(close)

    def tokens(self) -> list[Token]:
        out: list[Token] = []
        src, n = self.src, len(self.src)
        while True:
            # whitespace + comments
            while self.i < n:
                c = src[self.i]
                if c == "\n":
                    self.i += 1
                    self._newline(self.i)
                elif c in " \t\r\f\v":
                    self.i += 1
                elif src.startswith("--", self.i):
                    j = self.i + 2
                    level = self._long_level(j)
                    if level is not None:
                        self.i = self._skip_long(j, level, "comment")
                    else:
                        k = src.find("\n", j)
                        self.i = n if k < 0 else k
                else:
                    break
            start, line, col = self.i, self.line, self._col(self.i)
            if self.i >= n:
                out.append(Token("eof", "<eof>", line, col, n, n))
                return out
            c = src[self.i]
            if _is_name_start(c):
                j = self.i + 1
                while j < n and _is_name_char(src[j]):
                    j += 1
                word = src[self.i:j]
                self.i = j
                out.append(Token("keyword" if word in KEYWORDS else "name", word,
                                 line, col, start, j))
            elif c.isdigit() or (c == "." and self.i + 1 < n and src[self.i + 1].isdigit()):
                out.append(self._number(start, line, col))
            elif c in "\"'":
                out.append(self._string(start, line, col, c))
            elif c == "[" and self._long_level(self.i) is not None:
                level = self._long_level(self.i)
                end = self._skip_long(self.i, level, "string")
                self.i = end
                out.append(Token("string", src[start:end], line, col, start, end))
            else:
                for op in _OPS3 + _OPS2:
                    if src.startswith(op, self.i):
                        self.i += len(op)
                        out.append(Token("op", op, line, col, start, self.i))
                        break
                else:
                    if c in _OPS1:
                        self.i += 1
                        out.append(Token("op", c, line, col, start, self.i))
                    else:
                        raise self._err(f"unexpected symbol near '{c}'")

    def _number(self, start: int, line: int, col: int) -> Token:
        src, n = self.src, len(self.src)
        j = start
        exps = "pP" if src.startswith(("0x", "0X"), start) else "eE"
        while j < n:
            ch = src[j]
            if _is_name_char(ch) or ch == ".":
                j += 1
            elif ch in "+-" and src[j - 1] in exps:
                j += 1
            else:
                break
        text = src[start:j]
        if not (_DEC_RE.fullmatch(text) or _HEX_RE.fullmatch(text)):
            raise LuaSyntaxError(line, col, f"malformed number near '{text}'")
        self.i = j
        return Token("number", text, line, col, start, j)

    def _string(self, start: int, line: int, col: int, quote: str) -> Token:
        src, n = self.src, len(self.src)
        j = start + 1
        while True:
            if j >= n or src[j] == "\n":
                raise LuaSyntaxError(line, col, f"unfinished string near '{src[start:j][:40]}'")
            ch = src[j]
            if ch == quote:
                j += 1
                break
            if ch != "\\":
                j += 1
                continue
            j += 1
            if j >= n:
                raise LuaSyntaxError(line, col, "unfinished string")
            e = src[j]
            if e in "abfnrtv\\\"'":
                j += 1
            elif e == "\n":
                j += 1
                self._newline(j)
            elif e == "\r":
                j += 2 if src.startswith("\r\n", j) else 1
                self._newline(j)
            elif e == "x":
                if not re.fullmatch(r"[0-9a-fA-F]{2}", src[j + 1:j + 3]):
                    raise LuaSyntaxError(self.line, self._col(j), "hexadecimal digit expected")
                j += 3
            elif e == "z":
                j += 1
                while j < n and src[j] in " \t\r\n\f\v":
                    if src[j] == "\n":
                        self._newline(j + 1)
                    j += 1
            elif e.isdigit():
                k = j
                while k < n and k < j + 3 and src[k].isdigit():
                    k += 1
                if int(src[j:k]) > 255:
                    raise LuaSyntaxError(self.line, self._col(j), "decimal escape too large")
                j = k
            elif e == "u":
                m = re.match(r"\{[0-9a-fA-F]+\}", src[j + 1:])
                if not m:
                    raise LuaSyntaxError(self.line, self._col(j), "missing '{' / '}' in \\u{xxxx}")
                j += 1 + m.end()
            else:
                raise LuaSyntaxError(self.line, self._col(j), "invalid escape sequence")
        self.i = j
        return Token("string", src[start:j], line, col, start, j)


def tokenize(src: str) -> list[Token]:
    return _Lexer(src).tokens()


# ---- parser -----------------------------------------------------------------

@dataclass
class Stmt:
    line: int
    col: int
    pos: int
    end: int
    kind: str          # local | function | if | while | ... | call | assign | return | ...
    fn: int            # function id (0 = the chunk)
    level: int         # block nesting within that function (0 = its body)
    names: tuple = ()  # `local` statements: the declared names


@dataclass
class Call:
    name: str
    kind: str          # global | field | method
    line: int
    col: int
    pos: int
    fn: int
    base: str | None = None      # field calls `base.name(...)`: the receiver name
    base_kind: str | None = None  # ... and whether it is a global or a local


# Binary operator priorities (left, right), Lua 5.3 lparser.c.
_BINPRI = {
    "+": (10, 10), "-": (10, 10), "*": (11, 11), "%": (11, 11), "^": (14, 13),
    "/": (11, 11), "//": (11, 11), "&": (6, 6), "|": (4, 4), "~": (5, 5),
    "<<": (7, 7), ">>": (7, 7), "..": (9, 8),
    "==": (3, 3), "<": (3, 3), "<=": (3, 3), "~=": (3, 3), ">": (3, 3), ">=": (3, 3),
    "and": (2, 2), "or": (1, 1),
}
_UNARY_PRI = 12
_BLOCK_END = frozenset(("end", "else", "elseif", "until"))


class _Parser:
    def __init__(self, tokens: list[Token]) -> None:
        self.t = tokens
        self.i = 0
        self.scopes: list[set[str]] = [set()]
        self.statements: list[Stmt] = []
        self.calls: list[Call] = []
        self.global_defs: set[str] = set()
        self.loop_bodies: list[tuple[int, int, int]] = []   # (insert pos, line, fn)
        self.fn_parent: dict[int, int] = {0: -1}
        self.fn_vararg: dict[int, bool] = {0: True}
        self.fn_stack: list[int] = [0]
        self.level_stack: list[int] = [0]
        self.returned_fn: int | None = None   # chunk's `return function ... end`
        self.returned_name: str | None = None  # chunk's `return plugin`
        self.chunk_returns = False
        self.loop_depth: list[int] = [0]       # per function, for `break`
        self.labels: list[set[str]] = [set()]  # per function, for `goto`
        self.gotos: list[list[Token]] = [[]]
        self.aliases: dict[str, str] = {}      # local name -> require('aubo').<module>
        self.dynamic_globals: list[Token] = []  # `_G[...]` / `_ENV[...]`

    # -- token helpers --
    @property
    def tok(self) -> Token:
        return self.t[self.i]

    def peek(self, k: int = 1) -> Token:
        return self.t[min(self.i + k, len(self.t) - 1)]

    def next(self) -> Token:
        tok = self.t[self.i]
        if tok.kind != "eof":
            self.i += 1
        return tok

    def check(self, value: str) -> bool:
        tok = self.tok
        return tok.value == value and tok.kind in ("op", "keyword")

    def accept(self, value: str) -> bool:
        if self.check(value):
            self.next()
            return True
        return False

    def error(self, message: str, tok: Token | None = None) -> LuaSyntaxError:
        tok = tok or self.tok
        return LuaSyntaxError(tok.line, tok.col, f"{message} near {tok.near()}")

    def expect(self, value: str, opener: Token | None = None, what: str | None = None) -> Token:
        if self.check(value):
            return self.next()
        if opener is not None and opener.line != self.tok.line:
            raise self.error(f"'{value}' expected (to close '{what or opener.value}' "
                             f"at line {opener.line})")
        raise self.error(f"'{value}' expected")

    def expect_name(self) -> Token:
        if self.tok.kind != "name":
            raise self.error("<name> expected")
        return self.next()

    # -- scopes --
    def declare(self, name: str, shadow: bool = False) -> None:
        """`shadow`: the name is a parameter / loop variable / local
        function, so a module alias of that name no longer applies. (Aliases
        are a flat dict -- a shadow in an inner scope drops it for the rest
        of the chunk, which only costs a member check, never a false warning.)"""
        self.scopes[-1].add(name)
        if shadow:
            self.aliases.pop(name, None)

    def is_local(self, name: str) -> bool:
        return any(name in s for s in self.scopes)

    def open_block(self) -> None:
        self.scopes.append(set())
        self.level_stack[-1] += 1

    def close_block(self) -> None:
        self.scopes.pop()
        self.level_stack[-1] -= 1

    @property
    def fn(self) -> int:
        return self.fn_stack[-1]

    # -- grammar --
    def chunk(self) -> None:
        self.block(top=True)
        if self.tok.kind != "eof":
            raise self.error("'<eof>' expected")
        self._check_gotos()

    def block(self, top: bool = False) -> None:
        while True:
            tok = self.tok
            if tok.kind == "eof" or (tok.kind == "keyword" and tok.value in _BLOCK_END):
                return
            if tok.kind == "keyword" and tok.value == "return":
                self.retstat(top)
                return
            self.statement()

    def retstat(self, top: bool) -> None:
        start = self.next()
        stmt = self._open_stmt(start, "return")
        first_fn = first_name = None
        if not (self.tok.kind == "eof" or self.check(";")
                or (self.tok.kind == "keyword" and self.tok.value in _BLOCK_END)):
            if top and self.check("function"):
                first_fn = len(self.fn_parent)        # id the function will get
            elif top and self.tok.kind == "name" and (
                    self.peek().kind == "eof" or self.peek().value == ";"):
                first_name = self.tok.value
            self.explist()
        self.accept(";")
        self._close_stmt(stmt)
        if top:
            self.chunk_returns = True
            self.returned_fn = first_fn
            self.returned_name = first_name

    def _open_stmt(self, tok: Token, kind: str) -> Stmt:
        stmt = Stmt(tok.line, tok.col, tok.pos, tok.end, kind, self.fn, self.level_stack[-1])
        self.statements.append(stmt)
        return stmt

    def _close_stmt(self, stmt: Stmt) -> None:
        stmt.end = self.t[self.i - 1].end if self.i > 0 else stmt.end

    def statement(self) -> None:
        tok = self.tok
        if tok.kind == "op" and tok.value == ";":
            self.next()
            return
        if tok.kind == "op" and tok.value == "::":
            stmt = self._open_stmt(tok, "label")
            self.next()
            self.labels[-1].add(self.expect_name().value)
            self.expect("::")
            self._close_stmt(stmt)
            return
        kw = tok.value if tok.kind == "keyword" else None
        stmt = self._open_stmt(tok, kw or "expr")
        if kw == "if":
            self.next()
            self.expr()
            self.expect("then")
            self.open_block(); self.block(); self.close_block()
            while self.check("elseif"):
                self.next()
                self.expr()
                self.expect("then")
                self.open_block(); self.block(); self.close_block()
            if self.accept("else"):
                self.open_block(); self.block(); self.close_block()
            self.expect("end", tok, "if")
        elif kw == "while":
            self.next()
            self.expr()
            do = self.expect("do")
            self.loop_bodies.append((do.end, do.line, self.fn))
            self.open_block(); self._loop_block(); self.close_block()
            self.expect("end", tok, "while")
        elif kw == "do":
            self.next()
            self.open_block(); self.block(); self.close_block()
            self.expect("end", tok, "do")
        elif kw == "for":
            self.next()
            first = self.expect_name()
            names = [first.value]
            if self.accept("="):
                self.expr(); self.expect(","); self.expr()
                if self.accept(","):
                    self.expr()
            else:
                while self.accept(","):
                    names.append(self.expect_name().value)
                self.expect("in")
                self.explist()
            do = self.expect("do")
            self.loop_bodies.append((do.end, do.line, self.fn))
            self.open_block()
            for name in names:
                self.declare(name, shadow=True)
            self._loop_block()
            self.close_block()
            self.expect("end", tok, "for")
        elif kw == "repeat":
            rep = self.next()
            self.loop_bodies.append((rep.end, rep.line, self.fn))
            self.open_block()
            self._loop_block()
            self.expect("until", tok, "repeat")
            self.expr()              # sees the block's locals
            self.close_block()
        elif kw == "function":
            self.next()
            first = self.expect_name()
            is_method = False
            dotted = False
            while self.check(".") or self.check(":"):
                is_method = self.next().value == ":"
                dotted = True
                self.expect_name()
                if is_method:
                    break
            if not dotted:
                if self.is_local(first.value):
                    pass
                else:
                    self.global_defs.add(first.value)
            self.funcbody(tok, method=is_method)
        elif kw == "local":
            self.next()
            if self.accept("function"):
                name = self.expect_name()
                self.declare(name.value, shadow=True)
                self.funcbody(tok)
                stmt.kind = "local function"
            else:
                names = [self.expect_name().value]
                self._attrib()
                while self.accept(","):
                    names.append(self.expect_name().value)
                    self._attrib()
                module = None
                if self.accept("="):
                    start = self.i
                    self.explist()
                    module = self._aubo_module(self.t[start:self.i])
                for name in names:
                    self.declare(name)
                    self.aliases.pop(name, None)
                if module is not None and len(names) == 1:
                    self.aliases[names[0]] = module
                stmt.names = tuple(names)
        elif kw == "goto":
            self.next()
            self.gotos[-1].append(self.expect_name())
        elif kw == "break":
            if self.loop_depth[-1] == 0:
                raise LuaSyntaxError(tok.line, tok.col,
                                     f"break outside a loop at line {tok.line} near 'break'")
            self.next()
        elif kw is not None:
            raise self.error("unexpected symbol")
        else:
            self.exprstat(stmt)
        self._close_stmt(stmt)

    def _loop_block(self) -> None:
        self.loop_depth[-1] += 1
        self.block()
        self.loop_depth[-1] -= 1

    @staticmethod
    def _aubo_module(toks: list[Token]) -> str | None:
        """`require('aubo').<module>` exactly -> the module name (alias
        tracking for the member check), else None."""
        if len(toks) != 6:
            return None
        shape = [(t.kind, t.value) for t in toks[:5]]
        if shape[0] != ("name", "require") or shape[1] != ("op", "(") \
                or toks[2].kind != "string" or toks[2].value[1:-1] != "aubo" \
                or shape[3] != ("op", ")") or shape[4] != ("op", ".") or toks[5].kind != "name":
            return None
        return toks[5].value

    def _attrib(self) -> None:
        # Lua 5.4 `<const>` / `<close>` -- accepted (permissive).
        if self.check("<"):
            self.next()
            self.expect_name()
            self.expect(">")

    def exprstat(self, stmt: Stmt) -> None:
        kind, global_name = self.suffixedexp()
        if self.check("=") or self.check(","):
            targets = [(kind, global_name)]
            while self.accept(","):
                targets.append(self.suffixedexp())
            for target_kind, _ in targets:
                if target_kind not in ("var", "global"):
                    raise self.error("syntax error")
            self.expect("=")
            self.explist()
            for target_kind, name in targets:
                if target_kind == "global" and name:
                    self.global_defs.add(name)
            stmt.kind = "assign"
        else:
            if kind != "call":
                raise self.error("syntax error")
            stmt.kind = "call"

    def funcbody(self, opener: Token, method: bool = False) -> None:
        fn_id = len(self.fn_parent)
        self.fn_parent[fn_id] = self.fn
        self.fn_stack.append(fn_id)
        self.level_stack.append(0)
        self.loop_depth.append(0)
        self.labels.append(set())
        self.gotos.append([])
        self.scopes.append(set())
        if method:
            self.declare("self")
        self.expect("(")
        vararg = False
        if not self.check(")"):
            while True:
                if self.accept("..."):
                    vararg = True
                    break
                self.declare(self.expect_name().value, shadow=True)
                if not self.accept(","):
                    break
        self.fn_vararg[fn_id] = vararg
        self.expect(")")
        self.block()
        self.expect("end", opener, "function")
        self.scopes.pop()
        self.level_stack.pop()
        self.loop_depth.pop()
        self._check_gotos()
        self.fn_stack.pop()

    def _check_gotos(self) -> None:
        """Every goto needs a label in the same function (luac: "no visible
        label"). Visibility is per block in Lua; checking per function never
        rejects a valid program."""
        labels, gotos = self.labels.pop(), self.gotos.pop()
        for tok in gotos:
            if tok.value not in labels:
                raise LuaSyntaxError(tok.line, tok.col,
                                     f"no visible label '{tok.value}' for goto")

    def explist(self) -> None:
        self.expr()
        while self.accept(","):
            self.expr()

    def expr(self, limit: int = 0) -> None:
        tok = self.tok
        if (tok.kind == "keyword" and tok.value == "not") or (
                tok.kind == "op" and tok.value in ("-", "#", "~")):
            self.next()
            self.expr(_UNARY_PRI)
        else:
            self.simpleexp()
        while True:
            tok = self.tok
            op = tok.value if tok.kind in ("op", "keyword") else None
            pri = _BINPRI.get(op) if op else None
            if pri is None or pri[0] <= limit:
                return
            self.next()
            self.expr(pri[1])

    def simpleexp(self) -> None:
        tok = self.tok
        if tok.kind in ("number", "string"):
            self.next()
        elif tok.kind == "keyword" and tok.value in ("nil", "true", "false"):
            self.next()
        elif tok.kind == "op" and tok.value == "...":
            if not self.fn_vararg.get(self.fn, False):
                raise self.error("cannot use '...' outside a vararg function")
            self.next()
        elif tok.kind == "op" and tok.value == "{":
            self.table()
        elif tok.kind == "keyword" and tok.value == "function":
            self.next()
            self.funcbody(tok)
        else:
            self.suffixedexp()

    def primaryexp(self) -> tuple[str, str | None, Token]:
        tok = self.tok
        if tok.kind == "name":
            self.next()
            if self.is_local(tok.value):
                return "var", None, tok
            return "global", tok.value, tok
        if tok.kind == "op" and tok.value == "(":
            self.next()
            self.expr()
            self.expect(")", tok, "(")
            return "paren", None, tok
        raise self.error("unexpected symbol")

    def suffixedexp(self) -> tuple[str, str | None]:
        """Returns (kind, global name): kind is var | global | call | paren."""
        kind, global_name, tok = self.primaryexp()
        last_name: tuple[str, Token] | None = None   # (kind, name token) of a pending callee
        # `base.name(...)`: the receiver of a one-level field call, for the
        # module-member check (None once the chain is longer or was a call).
        base: tuple[str, str] | None = (tok.value, kind) if kind in ("global", "var") else None
        if kind == "global":
            last_name = ("global", tok)
        while True:
            cur = self.tok
            if cur.kind == "op" and cur.value == ".":
                self.next()
                name = self.expect_name()
                if last_name is not None and last_name[0] == "field":
                    base = None                      # a.b.c: deeper than one level
                kind, global_name, last_name = "var", None, ("field", name)
            elif cur.kind == "op" and cur.value == "[":
                if kind == "global" and global_name in ("_G", "_ENV"):
                    self.dynamic_globals.append(tok)
                self.next()
                self.expr()
                self.expect("]")
                kind, global_name, last_name, base = "var", None, None, None
            elif cur.kind == "op" and cur.value == ":":
                self.next()
                name = self.expect_name()
                self._record_call("method", name)
                self.args()
                kind, global_name, last_name, base = "call", None, None, None
            elif cur.kind == "string" or (cur.kind == "op" and cur.value in ("(", "{")):
                if last_name is not None:
                    self._record_call(*last_name, base=base if last_name[0] == "field" else None)
                self.args()
                kind, global_name, last_name, base = "call", None, None, None
            else:
                return kind, global_name

    def _record_call(self, kind: str, tok: Token, base: tuple[str, str] | None = None) -> None:
        self.calls.append(Call(tok.value, kind, tok.line, tok.col, tok.pos, self.fn,
                               base[0] if base else None, base[1] if base else None))

    def args(self) -> None:
        tok = self.tok
        if tok.kind == "string":
            self.next()
        elif tok.value == "{":
            self.table()
        else:
            # Lua 5.3: a call's '(' must be on the same line as its callee is
            # NOT enforced (5.2+ dropped "ambiguous syntax"); just parse.
            self.expect("(")
            if not self.check(")"):
                self.explist()
            self.expect(")", tok, "(")

    def table(self) -> None:
        opener = self.expect("{")
        while not self.check("}"):
            if self.check("["):
                self.next()
                self.expr()
                self.expect("]")
                self.expect("=")
                self.expr()
            elif self.tok.kind == "name" and self.peek().kind == "op" and self.peek().value == "=":
                self.next()
                self.next()
                self.expr()
            else:
                self.expr()
            if not (self.accept(",") or self.accept(";")):
                break
        self.expect("}", opener, "{")


# ---- API list ---------------------------------------------------------------

@dataclass(frozen=True)
class LuaApi:
    globals: dict            # name -> [param names]
    modules: dict            # module -> {name -> [param names]}
    source: dict

    @property
    def known(self) -> frozenset:
        return frozenset(self.globals) | LUA_GLOBALS

    def as_dict(self) -> dict:
        return {"globals": self.globals, "modules": self.modules,
                "lua_globals": sorted(LUA_GLOBALS), "source": self.source,
                "denied": DENIED, "caution": {k: v[1] for k, v in CAUTION.items()}}


@lru_cache(maxsize=1)
def default_api() -> LuaApi:
    try:
        data = json.loads(API_PATH.read_text())
    except (OSError, ValueError) as exc:
        # check() turns this into an error diagnostic (runs refused) rather
        # than silently flagging every AUBO call as unknown.
        log.error("AUBO Lua API catalog unusable (%s): %s -- program runs are refused "
                  "until it is restored (tools/fetch_aubo_lua_api.py)", API_PATH, exc)
        return LuaApi({}, {}, {"error": f"the AUBO API catalog {API_PATH.name} is missing or "
                                        f"unreadable ({exc}); run tools/fetch_aubo_lua_api.py"})
    return LuaApi(dict(data.get("globals", {})), dict(data.get("modules", {})),
                  dict(data.get("source", {})))


# ---- check ------------------------------------------------------------------

@dataclass
class Diagnostic:
    line: int
    col: int
    end_col: int
    severity: str        # error | warning
    code: str
    message: str
    fix: str | None = None       # replacement text for [col, end_col)

    def as_dict(self) -> dict:
        out = {"line": self.line, "col": self.col, "end_col": self.end_col,
               "severity": self.severity, "code": self.code, "message": self.message}
        if self.fix is not None:
            out["fix"] = self.fix
        return out


@dataclass
class Param:
    name: str
    value: float
    line: int
    unit: str
    min: float | None
    max: float | None
    integer: bool
    description: str
    value_start: int         # offsets of the numeric literal in the source
    value_end: int

    def as_dict(self) -> dict:
        return {"name": self.name, "value": self.value, "line": self.line,
                "unit": self.unit, "min": self.min, "max": self.max,
                "integer": self.integer, "description": self.description}


@dataclass
class CheckResult:
    diagnostics: list[Diagnostic] = field(default_factory=list)
    params: list[Param] = field(default_factory=list)
    line_plan: list[int] = field(default_factory=list)     # instrumented lines, in order
    exec_lines: list[int] = field(default_factory=list)    # statement lines in functions
    form: str | None = None      # "function" (return function(api)) | "plugin" (return app)
    syntax_ok: bool = False
    # instrumentation points (offsets), internal
    anchor_end: int | None = None
    stmt_points: list[tuple[int, int]] = field(default_factory=list)   # (pos, line)
    loop_points: list[int] = field(default_factory=list)

    @property
    def errors(self) -> list[Diagnostic]:
        return [d for d in self.diagnostics if d.severity == "error"]

    @property
    def warnings(self) -> list[Diagnostic]:
        return [d for d in self.diagnostics if d.severity == "warning"]

    @property
    def instrumentable(self) -> bool:
        return self.syntax_ok and self.anchor_end is not None

    def as_dict(self) -> dict:
        return {"ok": not self.errors, "syntax_ok": self.syntax_ok,
                "diagnostics": [d.as_dict() for d in self.diagnostics],
                "params": [p.as_dict() for p in self.params],
                "line_plan": self.line_plan, "form": self.form,
                "instrumentable": self.instrumentable}


def normalize_newlines(source: str) -> str:
    """CRLF / CR -> LF. The TCP script protocol ends a script at the first
    blank line (`\\r\\n\\r\\n`), so a CRLF file would be cut short."""
    return source.replace("\r\n", "\n").replace("\r", "\n")


_PARAM_LINE_RE = re.compile(
    r"^(?P<lead>\s*local\s+(?P<name>[A-Za-z_]\w*)\s*=\s*)"
    r"(?P<value>-?\s*(?:0[xX][0-9a-fA-F]+|\d+\.?\d*(?:[eE][+-]?\d+)?|\.\d+(?:[eE][+-]?\d+)?))"
    r"\s*;?\s*--\s*@param\b(?P<spec>.*)$")
_RANGE_RE = re.compile(r"^(-?\d+(?:\.\d+)?)\.\.(-?\d+(?:\.\d+)?)$")


def _parse_params(source: str, top_locals: set[int]) -> tuple[list[Param], list[Diagnostic]]:
    params: list[Param] = []
    diags: list[Diagnostic] = []
    offset = 0
    seen: set[str] = set()
    for number, text in enumerate(source.split("\n"), start=1):
        m = _PARAM_LINE_RE.match(text)
        if m:
            name = m.group("name")
            col = m.start("value") + 1
            if number not in top_locals:
                diags.append(Diagnostic(number, 1, len(text) + 1, "warning", "param",
                                        "@param only works on a top-level `local NAME = number` line"))
            elif name in seen:
                diags.append(Diagnostic(number, 1, len(text) + 1, "warning", "param",
                                        f"@param {name} is declared twice"))
            else:
                tokens = m.group("spec").split()
                unit, lo, hi = "", None, None
                if tokens and not _RANGE_RE.match(tokens[0]):
                    unit = tokens.pop(0)
                if tokens and _RANGE_RE.match(tokens[0]):
                    r = _RANGE_RE.match(tokens.pop(0))
                    lo, hi = float(r.group(1)), float(r.group(2))
                value = float(int(m.group("value").replace(" ", ""), 0)) if "x" in m.group(
                    "value").lower() else float(m.group("value").replace(" ", ""))
                integer = unit == "int"
                problem = None
                if lo is not None and lo > hi:
                    problem = f"range {lo:g}..{hi:g} is empty"
                elif lo is not None and not lo <= value <= hi:
                    problem = f"default {value:g} is outside {lo:g}..{hi:g}"
                elif integer and value != int(value):
                    problem = f"default {value:g} is not an integer"
                if problem:
                    diags.append(Diagnostic(number, col, col + len(m.group("value")),
                                            "warning", "param", f"@param {name}: {problem}"))
                seen.add(name)
                params.append(Param(name, value, number, unit, lo, hi, integer,
                                    " ".join(tokens), offset + m.start("value"),
                                    offset + m.end("value")))
        offset += len(text) + 1
    return params, diags


def _in_fn(parser: _Parser, fn: int, ancestor: int) -> bool:
    while fn != -1:
        if fn == ancestor:
            return True
        fn = parser.fn_parent.get(fn, -1)
    return False


def check(source: str, api: LuaApi | None = None) -> CheckResult:
    api = api or default_api()
    source = normalize_newlines(source)
    result = CheckResult()
    if len(source.encode()) > MAX_SOURCE_BYTES:
        result.diagnostics.append(Diagnostic(1, 1, 1, "error", "size",
                                             f"file is larger than {MAX_SOURCE_BYTES // 1024} KB"))
        return result
    if api.source.get("error"):
        result.diagnostics.append(Diagnostic(1, 1, 1, "error", "api-catalog",
                                             str(api.source["error"])))
    try:
        tokens = tokenize(source)
        parser = _Parser(tokens)
        parser.chunk()
    except LuaSyntaxError as exc:
        result.diagnostics.append(Diagnostic(exc.line, exc.col, exc.col + 1, "error",
                                             "syntax", exc.message))
        return result
    except RecursionError:
        # The recursive-descent parser's limit, as luac's LUAI_MAXCCALLS.
        result.diagnostics.append(Diagnostic(1, 1, 1, "error", "syntax",
                                             "chunk has too many syntax levels"))
        return result
    result.syntax_ok = True
    diags = result.diagnostics
    lines = source.split("\n")
    last_line = len(lines)

    # structure: AUBO's two documented forms -- `return function(api) ... end`
    # (TCP-script form, instrumentable) or `return plugin` (SDK examples:
    # plugin:start(api) builds the robot env with api:loadf).
    main = parser.returned_fn
    result.form = "function" if main is not None else (
        "plugin" if parser.returned_name else None)
    if result.form is None:
        diags.append(Diagnostic(last_line, 1, len(lines[-1]) + 1, "error", "structure",
                                "a program must end with `return function(api) ... end` "
                                "or `return <plugin table>` (AUBO's documented script forms)"))

    # The robot environment must be set up (select_robot, or loadf in the
    # plugin form) -- inside the returned function for the function form.
    robot_calls = [c for c in parser.calls if c.name == "select_robot" and c.kind != "global"
                   and (main is None or _in_fn(parser, c.fn, main))]
    env_calls = robot_calls + [c for c in parser.calls if c.name == "loadf"
                               and (main is None or _in_fn(parser, c.fn, main))]
    if result.form is not None and not env_calls:
        ret = next((s for s in reversed(parser.statements)
                    if s.fn == 0 and s.level == 0 and s.kind == "return"), None)
        line = ret.line if ret else 1
        diags.append(Diagnostic(line, 1, len(lines[line - 1]) + 1, "error", "no-robot-env",
                                "the program never calls require('aubo').sched.select_robot(1): "
                                "without it the robot API is not available, and calling it "
                                "anyway crashed the controller once (aubo-control-notes §10)"))
    result.exec_lines = sorted({s.line for s in parser.statements if s.fn != 0})

    # calls: operator-only, caution, unknown, print, module members
    known = api.known | parser.global_defs
    reported: set[tuple[int, int]] = set()      # (line, col) with a diagnostic already
    for call in parser.calls:
        end_col = call.col + len(call.name)
        if call.name in DENIED:
            reported.add((call.line, call.col))
            diags.append(Diagnostic(call.line, call.col, end_col, "error", "operator-only",
                                    f"{call.name}() is operator-only: {DENIED[call.name]}"))
            continue
        # Escape hatches are Lua globals: `x:load()` is some object's own
        # method. The AUBO cautions ARE methods in the full-API/plugin form
        # (`motion:setSpeedFraction(...)`), so they stay flagged either way.
        if call.name in CAUTION and (call.kind != "method" or CAUTION[call.name] is not _ESCAPE):
            code, why = CAUTION[call.name]
            diags.append(Diagnostic(call.line, call.col, end_col, "warning", code,
                                    f"{call.name}() {why}"))
            continue
        if call.kind == "field" and call.base is not None:
            _check_member(call, api, parser, diags)
            continue
        if call.kind != "global":
            continue
        if call.name == "print":
            diags.append(Diagnostic(call.line, call.col, end_col, "warning", "print-lost",
                                    "print() output never reaches the dashboard; use textmsg()",
                                    fix="textmsg"))
        elif call.name not in known:
            close = difflib.get_close_matches(call.name, sorted(known), n=1, cutoff=0.75)
            hint = f" Did you mean {close[0]}?" if close else ""
            diags.append(Diagnostic(call.line, call.col, end_col, "warning", "unknown-function",
                                    f"{call.name} is not a documented AUBO or Lua function."
                                    f"{hint} An undocumented call crashed the controller "
                                    "once (notes §10).", fix=close[0] if close else None))

    # operator-only names reached without a direct call: an alias
    # (`local p = poweroff`), `_G.poweroff` as a value, or a string naming one
    # (`_G['poweroff']`, `rawget(_G, 'poweroff')`, `load('poweroff()')`).
    for idx, tok in enumerate(tokens):
        if tok.kind == "name" and tok.value in DENIED and (tok.line, tok.col) not in reported:
            prev = tokens[idx - 1] if idx else None
            after = tokens[idx + 1] if idx + 1 < len(tokens) else None
            if after is not None and after.kind == "op" and after.value == "=":
                continue                 # a table key / assignment target of that name
            if prev is not None and (prev.kind == "keyword" and prev.value in ("function", "local")
                                     or prev.kind == "op" and prev.value == "::"):
                continue                 # defining / labelling something of that name
            via_dot = prev is not None and prev.kind == "op" and prev.value in (".", ":")
            if via_dot and not (idx >= 2 and tokens[idx - 2].kind == "name"
                                and tokens[idx - 2].value in ("_G", "_ENV")):
                continue                 # someone else's field of the same name
            reported.add((tok.line, tok.col))
            diags.append(Diagnostic(tok.line, tok.col, tok.col + len(tok.value), "error",
                                    "operator-only",
                                    f"{tok.value} is operator-only: {DENIED[tok.value]} "
                                    "(referencing it is refused, not only calling it)"))
        elif tok.kind == "string":
            body = _string_body(tok.value)
            if body in DENIED:
                diags.append(Diagnostic(tok.line, tok.col, tok.col + len(tok.value), "error",
                                        "operator-only",
                                        f"'{body}' names an operator-only call: {DENIED[body]}"))
            elif any(f"{name}(" in body for name in DENIED):
                diags.append(Diagnostic(tok.line, tok.col, tok.col + len(tok.value), "warning",
                                        "escape-hatch",
                                        "this string contains an operator-only call -- code "
                                        "built from strings bypasses the checker"))
    for tok in parser.dynamic_globals:
        diags.append(Diagnostic(tok.line, tok.col, tok.col + len(tok.value), "warning",
                                "escape-hatch",
                                f"indexing {tok.value}[...] reaches globals by name, which "
                                "bypasses the operator-only rules"))

    # params (top-level `local` statements of the chunk)
    top_locals = {s.line for s in parser.statements
                  if s.fn == 0 and s.level == 0 and s.kind == "local"}
    params, param_diags = _parse_params(source, top_locals)
    result.params = params
    diags.extend(param_diags)

    # instrumentation points: after the main-body `local _ENV = ...select_robot`
    # statement. The inserted setPlanContext / cancel_point calls are bare
    # names: anything else (`local env = ...`, select_robot in a helper) has no
    # _ENV for them to resolve in, so the program is not instrumentable.
    if main is not None and robot_calls:
        first = min(robot_calls, key=lambda c: c.pos)
        anchor = next((s for s in parser.statements
                       if s.fn == main and s.level == 0 and s.pos <= first.pos < s.end), None)
        if anchor is not None and not (anchor.kind == "local" and "_ENV" in anchor.names):
            diags.append(Diagnostic(
                first.line, first.col, first.col + len(first.name), "warning", "line-tracking",
                "live line tracking needs `local _ENV = require('aubo').sched.select_robot(1)` "
                "directly in the returned function; this program runs without it"))
            anchor = None
        if anchor is not None:
            result.anchor_end = anchor.end
            seen_lines: set[int] = set()
            for stmt in parser.statements:
                if stmt.pos < anchor.end or not _in_fn(parser, stmt.fn, main):
                    continue
                if stmt.line in seen_lines:
                    continue
                seen_lines.add(stmt.line)
                result.stmt_points.append((stmt.pos, stmt.line))
            result.line_plan = [line for _, line in sorted(result.stmt_points)]
            result.loop_points = [pos for pos, _, fn in parser.loop_bodies
                                  if pos > anchor.end and _in_fn(parser, fn, main)]
    diags.sort(key=lambda d: (d.line, d.col))
    return result


def _string_body(text: str) -> str:
    """Contents of a short or long string token (`'x'`, `"x"`, `[==[x]==]`)."""
    if text[0] in "\"'":
        return text[1:-1]
    level = text.index("[", 1)              # `[==[` -> 3
    return text[level + 1:-(level + 1)]


def _check_member(call: Call, api: LuaApi, parser: _Parser, diags: list) -> None:
    """`base.name(...)`: escape hatches and misspelt members of a known
    module -- an AUBO module bound with `local x = require('aubo').<module>`,
    or a Lua standard table used as a global."""
    end_col = call.col + len(call.name)
    if call.base_kind == "var":
        module = parser.aliases.get(call.base)
        members = api.modules.get(module) if module else None
        label = f"require('aubo').{module}"
    else:
        module = call.base
        if (module, call.name) in CAUTION_MEMBERS or (module, "*") in CAUTION_MEMBERS:
            code, why = CAUTION_MEMBERS.get((module, call.name)) or CAUTION_MEMBERS[(module, "*")]
            diags.append(Diagnostic(call.line, call.col, end_col, "warning", code,
                                    f"{module}.{call.name}() {why}"))
            return
        members = LUA_STD_MEMBERS.get(module)
        if members is not None and module in api.modules:
            members = members | frozenset(api.modules[module])   # AUBO extends `math`
        label = module
    if members is None or call.name in members:
        return
    close = difflib.get_close_matches(call.name, sorted(members), n=1, cutoff=0.75)
    hint = f" Did you mean {close[0]}?" if close else ""
    diags.append(Diagnostic(call.line, call.col, end_col, "warning", "unknown-member",
                            f"{call.base}.{call.name} is not a member of {label}.{hint}",
                            fix=close[0] if close else None))


def instrument(source: str, result: CheckResult | None = None) -> str:
    """The text sent to the controller when live line tracking is on (see the
    module doc). Requires a syntax-valid program with a select_robot anchor;
    otherwise the source is returned unchanged."""
    source = normalize_newlines(source)
    result = result or check(source)
    if not result.instrumentable:
        return source
    inserts: list[tuple[int, int, str]] = [(result.anchor_end, 0, _ANCHOR_TEXT)]
    inserts += [(pos, 1, _LINE_TEXT.format(line=line)) for pos, line in result.stmt_points]
    inserts += [(pos, 2, _CANCEL_TEXT) for pos in result.loop_points]
    out = source
    for pos, _, text in sorted(inserts, reverse=True):
        out = out[:pos] + text + out[pos:]
    return out


def apply_params(source: str, result: CheckResult, values: dict) -> str:
    """Rewrite the `@param` values. Raises ValueError (operator-facing message)
    on an unknown name, a non-number, out of range or a non-integer."""
    source = normalize_newlines(source)
    by_name = {p.name: p for p in result.params}
    unknown = sorted(set(values) - set(by_name))
    if unknown:
        raise ValueError(f"unknown parameter(s): {', '.join(unknown)}")
    edits = []
    for name, raw in values.items():
        p = by_name[name]
        try:
            value = float(raw)
        except (TypeError, ValueError):
            raise ValueError(f"{name}: {raw!r} is not a number") from None
        if not math.isfinite(value):
            raise ValueError(f"{name}: must be finite")
        if p.min is not None and not p.min <= value <= p.max:
            raise ValueError(f"{name}: {value:g} is outside {p.min:g}..{p.max:g}")
        if p.integer and value != int(value):
            raise ValueError(f"{name}: {value:g} must be an integer")
        text = str(int(value)) if p.integer else repr(value)
        edits.append((p.value_start, p.value_end, text))
    for start, end, text in sorted(edits, reverse=True):
        source = source[:start] + text + source[end:]
    return source
