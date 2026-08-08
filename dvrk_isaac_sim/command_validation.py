"""Validation and normalization for CRTK joint command messages."""

from __future__ import annotations

from typing import Iterable

import numpy as np


def joint_positions_from_message(message, expected_names: Iterable[str]) -> np.ndarray:
    """Return joint positions ordered according to the configured model."""
    expected = tuple(expected_names)
    if not message.position:
        raise ValueError("joint command has no position values")
    values = np.asarray(message.position, dtype=float)
    if message.name:
        names = tuple(message.name)
        positional_names = tuple(str(index) for index in range(len(expected)))
        if names == positional_names:
            # cisst's generic JointState bridge uses numeric names for an
            # ordered joint vector rather than the model-specific names.
            pass
        elif len(names) != len(set(names)) or set(names) != set(expected):
            raise ValueError(f"joint command names {names} do not match {expected}")
        else:
            values = np.asarray(
                [message.position[names.index(name)] for name in expected],
                dtype=float,
            )
    if values.shape != (len(expected),):
        raise ValueError("joint command has the wrong number of positions")
    return values


def jaw_position_from_message(message) -> float:
    """Return the single logical jaw position from a jaw command."""
    if not message.position:
        raise ValueError("jaw command has no position values")
    if len(message.position) != 1:
        raise ValueError("jaw command must contain exactly one position")
    return float(message.position[0])
