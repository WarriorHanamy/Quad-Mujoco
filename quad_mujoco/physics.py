"""Quadrotor physical description and helper utilities."""

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class QuadrotorPhysicalParams:
    """Physical constants for the quadrotor model."""

    gravity: float = 9.8066
    mass: float = 0.033
    inertia: Tuple[float, float, float] = (1.395e-5, 1.395e-5, 2.173e-5)
    thrust_coeff: float = 3.25e-4
    drag_coeff: float = 7.9379e-6
    arm_length: float = 0.065 / 2.0
    max_rotor_speed: float = 22.0
    max_thrust_per_motor: float = 0.1573
    max_torque_per_motor: float = 3.842e-3

    def inertia_matrix(self) -> np.ndarray:
        return np.diag(self.inertia)

    @property
    def total_thrust_limit(self) -> float:
        return 4.0 * self.max_thrust_per_motor

    @property
    def hover_speed(self) -> float:
        return math.sqrt((self.mass * self.gravity) / (4.0 * self.thrust_coeff))


QUAD_PARAMS = QuadrotorPhysicalParams()


def rotation_matrix_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    """Return the rotation matrix for a quaternion expressed as [w, x, y, z]."""
    w, x, y, z = quaternion
    return np.array(
        [
            [1.0 - 2.0 * (y**2 + z**2), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x**2 + z**2), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x**2 + y**2)],
        ]
    )


def quaternion_to_euler(quaternion: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] to roll, pitch, yaw (radians)."""
    w, x, y, z = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def allocation_matrix(params: QuadrotorPhysicalParams) -> np.ndarray:
    """Return the allocation matrix mapping squared rotor speeds to total wrench."""
    c = params.thrust_coeff
    cd = params.drag_coeff
    l = params.arm_length
    return np.array(
        [
            [c, c, c, c],
            [c * l, -c * l, -c * l, c * l],
            [-c * l, -c * l, c * l, c * l],
            [-cd, cd, -cd, cd],
        ]
    )


def solve_motor_speeds(total_thrust: float, torques: np.ndarray, params: QuadrotorPhysicalParams) -> np.ndarray:
    """Solve for rotor speeds (krpm) that achieve the desired wrench."""
    desired = np.concatenate(([total_thrust], torques))
    alloc = allocation_matrix(params)
    try:
        w_sq = np.linalg.solve(alloc, desired)
    except np.linalg.LinAlgError:
        w_sq, *_ = np.linalg.lstsq(alloc, desired, rcond=None)
    w_sq = np.clip(w_sq, 0.0, params.max_rotor_speed**2)
    return np.sqrt(w_sq)


def thrust_from_speed(params: QuadrotorPhysicalParams, speeds: np.ndarray) -> np.ndarray:
    return params.thrust_coeff * speeds**2


def normalize_thrust(params: QuadrotorPhysicalParams, thrust: np.ndarray) -> np.ndarray:
    return np.clip(thrust / params.max_thrust_per_motor, 0.0, 1.0)
