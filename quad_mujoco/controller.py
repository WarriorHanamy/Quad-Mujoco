"""Lightweight quadrotor controller that avoids ACADOS dependencies."""

from dataclasses import dataclass

import numpy as np

from .physics import (
    QUAD_PARAMS,
    QuadrotorPhysicalParams,
    normalize_thrust,
    quaternion_to_euler,
    solve_motor_speeds,
    thrust_from_speed,
)


@dataclass
class QuadrotorState:
    position: np.ndarray
    velocity: np.ndarray
    quaternion_wxyz: np.ndarray
    body_rates: np.ndarray


class SimpleQuadrotorController:
    """A basic cascaded position and attitude controller."""

    def __init__(self, params: QuadrotorPhysicalParams = QUAD_PARAMS):
        self.params = params
        self.inertia = np.array(params.inertia)
        self.kp_pos = np.array([1.5, 1.5, 6.0])
        self.kd_pos = np.array([1.0, 1.0, 3.0])
        self.kp_att = np.array([6.0, 6.0, 2.5])
        self.kd_att = np.array([0.3, 0.3, 0.15])
        self.max_tilt = np.radians(20.0)
        self.max_accel_xy = self.params.gravity * np.tan(self.max_tilt)

    def position_control(self, state: QuadrotorState, goal_position: np.ndarray) -> np.ndarray:
        pos_error = goal_position - state.position
        vel_error = -state.velocity

        desired_accel_xy = self.kp_pos[:2] * pos_error[:2] + self.kd_pos[:2] * vel_error[:2]
        desired_accel_xy = np.clip(desired_accel_xy, -self.max_accel_xy, self.max_accel_xy)

        desired_roll = desired_accel_xy[1] / self.params.gravity
        desired_pitch = -desired_accel_xy[0] / self.params.gravity
        desired_roll = np.clip(desired_roll, -self.max_tilt, self.max_tilt)
        desired_pitch = np.clip(desired_pitch, -self.max_tilt, self.max_tilt)
        desired_attitude = np.array([desired_roll, desired_pitch, 0.0])

        angles = quaternion_to_euler(state.quaternion_wxyz)
        att_error = desired_attitude - angles
        rate_error = -state.body_rates

        torque_cmd = self.inertia * (self.kp_att * att_error + self.kd_att * rate_error)

        desired_accel_z = self.kp_pos[2] * pos_error[2] + self.kd_pos[2] * vel_error[2]
        total_thrust = self.params.mass * (self.params.gravity + desired_accel_z)
        total_thrust = float(np.clip(total_thrust, 0.0, self.params.total_thrust_limit))

        rotor_speeds = solve_motor_speeds(total_thrust, torque_cmd, self.params)
        thrusts = thrust_from_speed(self.params, rotor_speeds)
        return normalize_thrust(self.params, thrusts)

    def state_from_vector(self, state_vector: np.ndarray) -> QuadrotorState:
        return QuadrotorState(
            position=state_vector[0:3],
            quaternion_wxyz=state_vector[3:7],
            velocity=state_vector[7:10],
            body_rates=state_vector[10:13],
        )

    def track_position(self, state_vector: np.ndarray, goal_position: np.ndarray) -> np.ndarray:
        state = self.state_from_vector(state_vector)
        return self.position_control(state, goal_position)


def build_state_vector(position: np.ndarray, quaternion_xyzw: np.ndarray, velocity: np.ndarray, omega: np.ndarray) -> np.ndarray:
    quaternion_wxyz = np.array([quaternion_xyzw[3], quaternion_xyzw[0], quaternion_xyzw[1], quaternion_xyzw[2]])
    return np.concatenate((position, quaternion_wxyz, velocity, omega))
