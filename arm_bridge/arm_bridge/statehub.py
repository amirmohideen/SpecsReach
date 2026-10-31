"""Latest-state fanout for the dashboard (SIM-DASHBOARD-PLAN §5, StateHub).

One producer, many readers, latest-wins. The producer is the transport's
existing 30 Hz `arm-status` thread: it already assembles `ServoLoop.status()`
for the Quest, so publishing the same snapshot here adds **zero** new
`read_snapshot()` callers (plan-review requirement). The dashboard server polls
`get()` at its own rate; a slow or absent dashboard can never back-pressure the
producer -- publish is a single reference swap under a short lock.

Snapshot contract (review requirements): `schema_version` + `boot_id` + `seq`
so a browser can detect bridge restarts and stale/out-of-order state; a
monotonic sample timestamp so the UI shows *sample age* rather than pretending
5 Hz robot reports are 30 Hz motion; joints carry names, not an anonymous
tuple (None until the sim/xarm backends publish them, Phase 2/4).

Robot model (multi-arm viewer, `robot_models.py`): every snapshot carries
`robot_model` -- the config override, else the id the ArmController reports via
`model_source` -- resolved through the catalog (aliases -> canonical id), and
`joints.names` are that model's URDF joint names, so the viewer and the
/arm/joint_states recorder speak the model's own joint vocabulary.
"""

from __future__ import annotations

import logging
import threading
import time
import uuid
from typing import Callable

from .robot_models import FALLBACK_JOINT_NAMES, RobotModelCatalog, joint_names_for

SCHEMA_VERSION = 1

log = logging.getLogger("arm_bridge.statehub")

# Names used when no catalog model is active (the pre-catalog behavior).
JOINT_NAMES = FALLBACK_JOINT_NAMES


class StateHub:
    """Thread-safe latest-snapshot holder.

    `publish()` takes the already-built wire Status plus session/backend context
    and stamps seq + timestamps; `get()` returns the latest dict (or None before
    the first publish). Dicts are treated as immutable once published.
    """

    def __init__(self, backend: str, servo_hz: float, status_hz: float,
                 clock=time.monotonic, wall_clock=time.time,
                 catalog: RobotModelCatalog | None = None,
                 model_source: Callable[[], str | None] | None = None,
                 model_override: str = "",
                 power_source: Callable[[], dict | None] | None = None) -> None:
        self.boot_id = uuid.uuid4().hex
        self._backend = backend
        self._servo_hz = servo_hz
        self._status_hz = status_hz
        self._clock = clock
        self._wall_clock = wall_clock
        # Robot model: `model_override` (config robot_model) wins, else
        # `model_source()` -- ArmController.robot_model, a cheap cached read
        # (contract) since it runs on the status thread every publish.
        self._catalog = catalog if catalog is not None else RobotModelCatalog()
        self._model_source = model_source
        self._model_override = model_override
        # Arm power detail (ArmController.power_info, backends with
        # supports_power): the same cheap-cached-read contract.
        self._power_source = power_source
        self._model_cache: tuple[str | None, object] = (None, None)
        self._lock = threading.Lock()
        self._seq = 0
        self._state: dict | None = None

    def _resolve_model(self):
        """(wire id, RobotModel | None) for the current model. A known id or
        alias maps to the canonical id; an unknown id is passed through as-is
        so the viewer can say which model is missing. Producer thread only."""
        model_id = self._model_override or (
            self._model_source() if self._model_source is not None else None) or None
        cached_id, cached = self._model_cache
        if model_id != cached_id:
            cached = self._catalog.resolve(model_id)
            self._model_cache = (model_id, cached)
        return (cached.id if cached is not None else model_id), cached

    def _power(self) -> dict | None:
        if self._power_source is None:
            return None
        try:
            return self._power_source()
        except Exception:  # noqa: BLE001 -- a backend bug must not stop the status thread
            return None

    @staticmethod
    def _link(session) -> dict | None:
        monitor = getattr(session, "link", None)
        if monitor is None:
            return None
        try:
            return monitor.snapshot()
        except Exception as exc:  # noqa: BLE001 -- a stats bug must not stop the status thread
            if not getattr(StateHub, "_link_err_logged", False):
                StateHub._link_err_logged = True
                log.warning("statehub: link snapshot failed (once): %r", exc)
            return None

    def publish(self, status, session, joints: tuple | None = None,
                envelope: str | None = None) -> None:
        """Build + swap the latest snapshot. Called from the status thread only
        (single producer); must stay cheap and non-blocking.

        `status` is a `protocol.Status`; `session` is the live `Session` (only
        GIL-atomic/property reads are taken from it, no session lock is held
        across this call).
        """
        now_mono = self._clock()
        hb_age_ms = session.hb_age_ms(now_mono)
        model_id, model = self._resolve_model()
        state = {
            "schema_version": SCHEMA_VERSION,
            "boot_id": self.boot_id,
            "backend": self._backend,
            "servo_hz": self._servo_hz,
            "status_hz": self._status_hz,
            "ts_wall_ms": self._wall_clock() * 1000.0,
            "ts_mono": now_mono,
            "arm": {
                "state": int(status.arm_state),
                "state_name": status.arm_state.name,
                "moving": bool(status.moving),
                "servo_active": bool(status.servo_active),
                "protective_stop": bool(status.protective_stop),
                "pose": [status.x, status.y, status.z,
                         status.roll, status.pitch, status.yaw],
                "gripper": status.gripper,
                "error_code": status.error_code,
                "warn_code": status.warn_code,
                "error_is_api": bool(status.error_is_api),
                "hold_latched": bool(status.hold_latched),
                # Why the actual posture holds teleop (bridge envelope), or null.
                "envelope": envelope,
                # Power phase / controller modes / last power-up error, or null.
                "power": self._power(),
            },
            "robot_model": model_id,
            "joints": ({"names": joint_names_for(model, len(joints)),
                        "degrees": list(joints)}
                       if joints is not None else None),
            "session": {
                "connected": session.established,
                "enabled": session.enabled_desired,
                "deadman": session.deadman_held,
                "hb_age_ms": hb_age_ms,
                "accepted": session.accepted_count,
                "rejected": session.rejected_count,
                "seq_skips": session.seq_skips,
                "hb_gap_max_ms": session.hb_gap_max_ms,
            },
            # Teleop link quality (link_stats.LinkMonitor): cached, recomputed
            # at most 4x/s, so publishing it at 30 Hz stays cheap.
            "link": self._link(session),
        }
        with self._lock:
            self._seq += 1
            state["seq"] = self._seq
            self._state = state

    def get(self) -> dict | None:
        """Latest snapshot (the published dict itself -- treat as read-only)."""
        with self._lock:
            return self._state

    def age_s(self, now: float | None = None) -> float | None:
        """Seconds since the last publish, for producer-staleness detection
        (e.g. the status thread died); None before the first publish."""
        with self._lock:
            state = self._state
        if state is None:
            return None
        return (self._clock() if now is None else now) - state["ts_mono"]
