"""Configuration loading for backend-independent robot models."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml


@dataclass(frozen=True)
class JointConfig:
    name: str
    type: str
    lower: float
    upper: float
    velocity: float


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(result.get(key), dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_robot_document(path: str | Path, _stack: tuple[Path, ...] = ()) -> dict[str, Any]:
    """Load a robot YAML document and resolve its relative ``include`` files."""
    source = Path(path).expanduser().resolve()
    if source in _stack:
        chain = " -> ".join(str(item) for item in (*_stack, source))
        raise ValueError(f"cyclic robot YAML include: {chain}")
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError(f"{source}: expected a YAML mapping")
    includes = document.pop("include", [])
    if isinstance(includes, (str, Path)):
        includes = [includes]
    if not isinstance(includes, list):
        raise ValueError(f"{source}: include must be a path or list of paths")
    merged: dict[str, Any] = {}
    for include in includes:
        include_path = Path(include)
        if not include_path.is_absolute():
            include_path = source.parent / include_path
        merged = _deep_merge(merged, load_robot_document(include_path, (*_stack, source)))
    return _deep_merge(merged, document)


@dataclass(frozen=True)
class RobotConfig:
    name: str
    type: str
    parent_frame: str
    base_frame: str
    rcm_frame: str
    tool_frame: str
    adaptor_frame: str
    base_position: np.ndarray
    base_orientation_xyzw: np.ndarray
    joints: tuple[JointConfig, ...]
    home_position: np.ndarray
    raw: dict[str, Any]
    kinematics_manifest: Path | None = None


def load_robot_config(path: str | Path, kinematics_manifest: str | Path | None = None,
                      base_position: Any | None = None,
                      base_orientation_xyzw: Any | None = None) -> RobotConfig:
    """Load and validate one robot YAML configuration."""

    source = Path(path)
    document = load_robot_document(source)

    if not isinstance(document, dict) or not isinstance(document.get("robot"), dict):
        raise ValueError(f"{source}: expected a top-level 'robot' mapping")

    robot = document["robot"]
    robot_type = str(robot["type"]).upper() if "type" in robot else ""
    if robot_type not in {"PSM", "ECM"}:
        raise ValueError(f"{source}: robot.type must be PSM or ECM")
    robot["type"] = robot_type
    required = ("name", "type", "parent_frame", "base_frame", "rcm_frame", "tool_frame", "adaptor_frame", "joints", "home_position")
    missing = [key for key in required if key not in robot]
    if missing:
        raise ValueError(f"{source}: missing robot fields: {', '.join(missing)}")

    base_pose = robot.get("base_pose", {})
    position = np.asarray(base_pose.get("position", [0.0, 0.0, 0.0]), dtype=float)
    orientation = np.asarray(base_pose.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]), dtype=float)
    if position.shape != (3,):
        raise ValueError(f"{source}: base_pose.position must have three values")
    if orientation.shape != (4,):
        raise ValueError(f"{source}: base_pose.orientation_xyzw must have four values")
    if np.linalg.norm(orientation) == 0.0:
        raise ValueError(f"{source}: base_pose.orientation_xyzw cannot be zero")

    joints = []
    for joint in robot["joints"]:
        try:
            item = JointConfig(
                name=str(joint["name"]),
                type=str(joint["type"]),
                lower=float(joint["lower"]),
                upper=float(joint["upper"]),
                velocity=float(joint["velocity"]),
            )
        except KeyError as error:
            raise ValueError(f"{source}: joint missing field {error.args[0]}") from error
        if item.type not in {"revolute", "prismatic"}:
            raise ValueError(f"{source}: unsupported joint type {item.type!r}")
        if item.lower > item.upper or item.velocity <= 0.0:
            raise ValueError(f"{source}: invalid limits for joint {item.name!r}")
        joints.append(item)

    home = np.asarray(robot["home_position"], dtype=float)
    if home.shape != (len(joints),):
        raise ValueError(f"{source}: home_position must match the joint count")
    for value, joint in zip(home, joints):
        if not joint.lower <= value <= joint.upper:
            raise ValueError(f"{source}: home position exceeds limits for {joint.name!r}")

    if base_position is not None:
        position = np.asarray(base_position, dtype=float)
        if position.shape != (3,):
            raise ValueError(f"{source}: overridden base position must have three values")
    if base_orientation_xyzw is not None:
        orientation = np.asarray(base_orientation_xyzw, dtype=float)
        if orientation.shape != (4,) or np.linalg.norm(orientation) == 0.0:
            raise ValueError(f"{source}: overridden base orientation must be a non-zero quaternion")

    return RobotConfig(
        name=str(robot["name"]),
        type=robot_type,
        parent_frame=str(robot["parent_frame"]),
        base_frame=str(robot["base_frame"]),
        rcm_frame=str(robot["rcm_frame"]),
        tool_frame=str(robot["tool_frame"]),
        adaptor_frame=str(robot["adaptor_frame"]),
        base_position=position,
        base_orientation_xyzw=orientation / np.linalg.norm(orientation),
        joints=tuple(joints),
        home_position=home,
        raw=document,
        kinematics_manifest=(Path(kinematics_manifest).expanduser().resolve()
                             if kinematics_manifest is not None else None),
    )
