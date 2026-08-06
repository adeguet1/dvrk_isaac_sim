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


def load_robot_config(path: str | Path) -> RobotConfig:
    """Load and validate one robot YAML configuration."""

    source = Path(path)
    with source.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    if not isinstance(document, dict) or not isinstance(document.get("robot"), dict):
        raise ValueError(f"{source}: expected a top-level 'robot' mapping")

    robot = document["robot"]
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

    return RobotConfig(
        name=str(robot["name"]),
        type=str(robot["type"]),
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
    )
