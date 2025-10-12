"""Quad MuJoCo package exposing controller and physics helpers."""

from .controller import SimpleQuadrotorController, QuadrotorState
from .se3_controller import SE3Controller, SE3State, create_default_se3_controller, state_from_vector
from .motor_mixer import MotorMixer, create_default_mixer
from .geometry_se3 import GeoQuaternion, veemap, hatmap, quaternion_xyzw_to_wxyz, quaternion_wxyz_to_xyzw
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
    "SE3Controller",
    "SE3State",
    "MotorMixer",
    "GeoQuaternion",
    "QUAD_PARAMS",
    "QuadrotorPhysicalParams",
    "create_default_se3_controller",
    "create_default_mixer",
    "state_from_vector",
    "veemap",
    "hatmap",
    "quaternion_xyzw_to_wxyz",
    "quaternion_wxyz_to_xyzw",
    "allocation_matrix",
    "normalize_thrust",
    "quaternion_to_euler",
    "rotation_matrix_from_quaternion",
    "solve_motor_speeds",
    "thrust_from_speed",
]
