"""Robot-model catalog for the dashboard 3D viewer (multi-arm support).

Every vendored model is one directory under `dashboard/static/assets/<id>/`
(URDF + meshes + license record + MANIFEST.json, produced by
`tools/fetch_urdf.sh <id>`). `MANIFEST.json["model"]` is the runtime contract:

    {"id": "aubo_i5H_10", "label": "AUBO i5H", "urdf": "aubo_i5H_10.urdf",
     "joint_names": ["shoulder_joint", ...],   # i-th name <- i-th backend joint
     "tcp_link": "wrist3_Link",
     "gripper": null | {"joint", "open_mm", "open_rad", "closed_rad"},
     "aliases": ["aubo_i5H"],
     "backends": ["aubo"]}                     # arm_backend values that drive it

The arm type is CONFIG: `robot_model` in arm_bridge.yaml, next to
`arm_backend`, which must be one of the model's `backends` (validated; the mock
accepts any model, for previews). Empty = the driver's default from
`ArmController.robot_model()` (mock/sim/xarm: uf850). A real driver verifies at
connect that the robot matches (xarm: expected_model; aubo: getRobotType() +
getRobotSubType() = 'aubo_i5H' + 10 -> 'aubo_i5H_10'). The StateHub stamps the
resolved id onto every snapshot; the browser fetches the descriptor
(`/api/robot-models/<id>`) and loads the URDF -- adding an arm is a vendored
directory, no frontend code.

Stdlib only: imported by config/statehub on every backend, never needs the
dashboard deps.
"""

from __future__ import annotations

import functools
import json
import math
from dataclasses import dataclass
from pathlib import Path

MODELS_DIR = Path(__file__).parent / "dashboard" / "static" / "assets"
# URL prefix the dashboard serves MODELS_DIR under (StaticFiles mount at "/").
ASSETS_URL_PREFIX = "/assets"
# Joint names stamped on a snapshot when no model (or a model with a different
# joint count) is active -- the pre-catalog behavior.
FALLBACK_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


@dataclass(frozen=True)
class RobotModel:
    id: str
    label: str
    urdf: str
    joint_names: tuple[str, ...]
    tcp_link: str | None = None
    gripper: dict | None = None
    aliases: tuple[str, ...] = ()
    backends: tuple[str, ...] = ()
    # Per joint_names entry, from the URDF joint table in the manifest; None
    # when a joint has no limits there.
    joint_limits_deg: tuple[tuple[float, float], ...] | None = None

    def drivable_by(self, backend: str) -> bool:
        """Whether `arm_backend` can drive this model. The mock drives nothing
        real, so it can stand in for any model (UI previews)."""
        return backend == "mock" or backend in self.backends

    @property
    def urdf_url(self) -> str:
        return f"{ASSETS_URL_PREFIX}/{self.id}/{self.urdf}"

    def as_dict(self) -> dict:
        """Wire descriptor served to the viewer."""
        return {
            "id": self.id,
            "label": self.label,
            "urdf_url": self.urdf_url,
            "joint_names": list(self.joint_names),
            "tcp_link": self.tcp_link,
            "gripper": dict(self.gripper) if self.gripper else None,
            "aliases": list(self.aliases),
            "backends": list(self.backends),
            "joint_limits_deg": ([list(pair) for pair in self.joint_limits_deg]
                                 if self.joint_limits_deg else None),
        }


def joint_names_for(model: RobotModel | None, count: int) -> list[str]:
    """Names for `count` backend joint angles: the model's when it matches the
    joint count, else generic joint1..jointN."""
    if model is not None and len(model.joint_names) == count:
        return list(model.joint_names)
    if count == len(FALLBACK_JOINT_NAMES):
        return list(FALLBACK_JOINT_NAMES)
    return [f"joint{i + 1}" for i in range(count)]


def _is_number(value) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def parse_model(model_dir: Path, manifest: dict) -> RobotModel:
    """Validate one MANIFEST.json's `model` block; ValueError on any defect."""
    block = manifest.get("model") if isinstance(manifest, dict) else None
    if not isinstance(block, dict):
        raise ValueError("no 'model' block (re-vendor with tools/fetch_urdf.sh)")
    model_id = block.get("id")
    if model_id != model_dir.name:
        raise ValueError(f"model id {model_id!r} != directory name {model_dir.name!r}")
    label = block.get("label")
    if not isinstance(label, str) or not label:
        raise ValueError("label must be a non-empty string")
    urdf = block.get("urdf")
    if (not isinstance(urdf, str) or not urdf or "/" in urdf or "\\" in urdf
            or urdf.startswith(".")):
        raise ValueError(f"urdf must be a bare file name (got {urdf!r})")
    if not (model_dir / urdf).is_file():
        raise ValueError(f"urdf file {urdf!r} missing")
    names = block.get("joint_names")
    if (not isinstance(names, list) or not names
            or not all(isinstance(n, str) and n for n in names)
            or len(set(names)) != len(names)):
        raise ValueError("joint_names must be a non-empty list of unique strings")
    tcp_link = block.get("tcp_link")
    if tcp_link is not None and not (isinstance(tcp_link, str) and tcp_link):
        raise ValueError("tcp_link must be a non-empty string or null")
    gripper = block.get("gripper")
    if gripper is not None:
        if not (isinstance(gripper, dict)
                and isinstance(gripper.get("joint"), str) and gripper["joint"]
                and _is_number(gripper.get("open_mm")) and gripper["open_mm"] > 0
                and _is_number(gripper.get("open_rad"))
                and _is_number(gripper.get("closed_rad"))):
            raise ValueError("gripper must be null or {joint, open_mm>0, "
                             "open_rad, closed_rad}")
    aliases = block.get("aliases", [])
    if not isinstance(aliases, list) or not all(isinstance(a, str) and a for a in aliases):
        raise ValueError("aliases must be a list of non-empty strings")
    backends = block.get("backends")
    if not isinstance(backends, list) or not all(isinstance(b, str) and b for b in backends):
        raise ValueError("backends must be a list of arm_backend names "
                         "(re-vendor with tools/fetch_urdf.sh)")
    return RobotModel(id=model_id, label=label, urdf=urdf,
                      joint_names=tuple(names), tcp_link=tcp_link,
                      gripper=dict(gripper) if gripper else None,
                      aliases=tuple(aliases), backends=tuple(backends),
                      joint_limits_deg=_joint_limits(manifest, names))


def _joint_limits(manifest: dict, names) -> tuple[tuple[float, float], ...] | None:
    """URDF limits (rad, MANIFEST arm_joints table) for `names`, in degrees."""
    table = {j.get("name"): j for j in manifest.get("arm_joints") or []
             if isinstance(j, dict)}
    limits = []
    for name in names:
        joint = table.get(name, {})
        lo, hi = joint.get("lower"), joint.get("upper")
        if not (_is_number(lo) and _is_number(hi) and lo < hi):
            return None
        limits.append((round(math.degrees(lo), 3), round(math.degrees(hi), 3)))
    return tuple(limits)


class RobotModelCatalog:
    """Immutable id -> RobotModel map. A broken vendored directory is skipped
    and reported in `errors` (the viewer is observability -- it must not take
    the bridge down), and the test suite asserts the committed set is clean."""

    def __init__(self, models: dict[str, RobotModel] | None = None,
                 errors: list[str] | None = None) -> None:
        self._models = dict(models or {})
        self.errors = list(errors or [])
        # Lookup keys: id and aliases, case-folded (AUBO itself is
        # inconsistent: aubo_i5H_10 vs aubo_i5h_30 upstream).
        self._index: dict[str, RobotModel] = {}
        for model in self._models.values():
            for key in (model.id, *model.aliases):
                self._index.setdefault(key.casefold(), model)

    @classmethod
    def load(cls, root: Path = MODELS_DIR) -> "RobotModelCatalog":
        models: dict[str, RobotModel] = {}
        errors: list[str] = []
        for manifest_path in sorted(root.glob("*/MANIFEST.json")):
            model_dir = manifest_path.parent
            try:
                manifest = json.loads(manifest_path.read_text())
                models[model_dir.name] = parse_model(model_dir, manifest)
            except (OSError, ValueError) as exc:     # JSONDecodeError is a ValueError
                errors.append(f"{model_dir.name}: {exc}")
        return cls(models, errors)

    def ids(self) -> list[str]:
        return sorted(self._models)

    def models(self) -> list[RobotModel]:
        return [self._models[i] for i in self.ids()]

    def resolve(self, model_id: str | None) -> RobotModel | None:
        """Model for an id or alias (case-insensitive); None if unknown."""
        if not model_id:
            return None
        return self._index.get(model_id.casefold())


@functools.lru_cache(maxsize=1)
def default_catalog() -> RobotModelCatalog:
    """The committed catalog, scanned once per process."""
    return RobotModelCatalog.load()
