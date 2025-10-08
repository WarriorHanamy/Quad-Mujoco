"""Quad MuJoCo package exposing controller and physics helpers."""

from .controller import SimpleQuadrotorController, QuadrotorState
from .physics import (
    QUAD_PARAMS,
    QuadrotorPhysicalParams,
    allocation_matrix,
    normalize_thrust,
    quaternion_to_euler,
    rotation_matrix_from_quaternion,
    solve_motor_speeds,
    thrust_from_speed,
)

__all__ = [
    "SimpleQuadrotorController",
    "QuadrotorState",
    "QUAD_PARAMS",
    "QuadrotorPhysicalParams",
    "allocation_matrix",
    "normalize_thrust",
    "quaternion_to_euler",
    "rotation_matrix_from_quaternion",
    "solve_motor_speeds",
    "thrust_from_speed",
]
