"""SE3 geometric controller for quadrotor position control.

This module implements an SE(3) based geometric controller that provides
mathematically rigorous position and attitude control for quadrotors.
Based on the Quadrotor_SE3_Control repository implementation.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional

from .geometry_se3 import GeoQuaternion, veemap
from .physics import QuadrotorPhysicalParams


@dataclass
class SE3State:
    """State representation for SE3 controller.

    Quaternion order: x, y, z, w (scalar last)
    """

    position: np.ndarray  # [x, y, z]
    velocity: np.ndarray  # [vx, vy, vz]
    quaternion: np.ndarray  # [x, y, z, w]
    omega: np.ndarray  # [wx, wy, wz] - angular velocity in body frame

    def __post_init__(self):
        """Validate state dimensions."""
        assert self.position.shape == (3,)
        assert self.velocity.shape == (3,)
        assert self.quaternion.shape == (4,)
        assert self.omega.shape == (3,)

    def update(self, pos: np.ndarray, vel: np.ndarray, quat: np.ndarray, omega: np.ndarray):
        """Update state with new values."""
        self.position = pos
        self.velocity = vel
        self.quaternion = quat
        self.omega = omega


@dataclass
class ControlCommand:
    """Control command output from SE3 controller."""

    thrust: float  # Total thrust command (in g units)
    angular: np.ndarray  # [wx, wy, wz] angular torque commands


class SE3Controller:
    """SE(3) geometric controller for quadrotor position control.

    This controller implements geometric control on the SE(3) manifold,
    providing global asymptotic stability for position and attitude control.
    """

    def __init__(self, params: QuadrotorPhysicalParams):
        """Initialize SE3 controller.

        Args:
            params: Quadrotor physical parameters
        """
        self.params = params
        self.goal_state: Optional[SE3State] = None
        self.current_state: Optional[SE3State] = None

        # Control gains
        self.kx = 0.6  # Position control gain
        self.kv = 0.4  # Velocity control gain
        self.kR = 6.0  # SO(3) attitude control gain
        self.kw = 1.0  # Angular velocity control gain

        # Gravity vector in world frame
        self.gravity = np.array([0.0, 0.0, -1.0])

    def set_current_state(self, state: SE3State):
        """Set current quadrotor state.

        Args:
            state: Current quadrotor state
        """
        self.current_state = state

    def set_goal_state(self, state: SE3State):
        """Set desired goal state.

        Args:
            state: Desired goal state
        """
        self.goal_state = state

    def set_gains(self, kx: float = None, kv: float = None, kR: float = None, kw: float = None):
        """Set controller gains.

        Args:
            kx: Position control gain
            kv: Velocity control gain
            kR: Attitude control gain
            kw: Angular velocity control gain
        """
        if kx is not None:
            self.kx = kx
        if kv is not None:
            self.kv = kv
        if kR is not None:
            self.kR = kR
        if kw is not None:
            self.kw = kw

    def update_linear_error(self) -> tuple[np.ndarray, np.ndarray]:
        """Calculate position and velocity errors.

        Returns:
            Tuple of (position_error, velocity_error)
        """
        if self.goal_state is None or self.current_state is None:
            raise ValueError("Goal or current state is None")

        # Position error: current - goal
        e_x = self.current_state.position - self.goal_state.position

        # Velocity error: current - goal
        e_v = self.current_state.velocity - self.goal_state.velocity

        return e_x, e_v

    def update_angular_error(self, trans_control: np.ndarray, forward: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        """Calculate attitude and angular velocity errors.

        Args:
            trans_control: Translational control acceleration vector
            forward: Desired forward direction (heading) vector

        Returns:
            Tuple of (attitude_error, angular_velocity_error, thrust_command)
        """
        if self.goal_state is None or self.current_state is None:
            raise ValueError("Goal or current state is None")

        # Get current rotation matrix
        q_curr = GeoQuaternion(
            self.current_state.quaternion[0],
            self.current_state.quaternion[1],
            self.current_state.quaternion[2],
            self.current_state.quaternion[3]
        )
        R_curr = q_curr.getRotationMatrix()

        # Calculate desired body z-axis direction
        goal_z = trans_control - self.gravity
        goal_z_norm = np.linalg.norm(goal_z)

        if goal_z_norm > 1e-6:
            goal_z = goal_z / goal_z_norm
        else:
            goal_z = R_curr[:, 2]  # Keep current z-axis

        # Construct desired rotation matrix
        up = goal_z
        right_des = np.cross(forward, up)
        right_des_norm = np.linalg.norm(right_des)

        if right_des_norm > 1e-6:
            right_des = right_des / right_des_norm
        else:
            # If forward and up are parallel, choose an arbitrary right direction
            if abs(up[2]) < 0.9:
                right_des = np.cross([0, 0, 1], up)
            else:
                right_des = np.cross([1, 0, 0], up)
            right_des = right_des / np.linalg.norm(right_des)

        proj_fwd_des = np.cross(up, right_des)

        R_goal = np.zeros((3, 3))
        R_goal[:, 0] = right_des
        R_goal[:, 1] = proj_fwd_des
        R_goal[:, 2] = up

        thrust = goal_z_norm

        # Calculate SO(3) attitude error using vee map
        e_R = 0.5 * veemap(np.dot(R_goal.T, R_curr) - np.dot(R_curr.T, R_goal))

        # Calculate angular velocity error
        w_curr = self.current_state.omega
        w_des = self.goal_state.omega
        e_w = w_curr - np.dot(R_curr.T, np.dot(R_goal, w_des))

        return e_R, e_w, thrust

    def control_update(self, current_state: SE3State, goal_state: SE3State,
                      dt: float, forward: Optional[np.ndarray] = None) -> ControlCommand:
        """Main control update function.

        Args:
            current_state: Current quadrotor state
            goal_state: Desired goal state
            dt: Time step (unused, kept for compatibility)
            forward: Desired forward direction. If None, uses goal state quaternion

        Returns:
            Control command with thrust and angular torques
        """
        self.current_state = current_state
        self.goal_state = goal_state

        # Calculate linear errors
        e_x, e_v = self.update_linear_error()

        # Position and velocity control (linear control)
        x = -self.kx * e_x[0] - self.kv * e_v[0] + self.goal_state.velocity[0]
        y = -self.kx * e_x[1] - self.kv * e_v[1] + self.goal_state.velocity[1]
        z = -self.kx * e_x[2] - self.kv * e_v[2] + self.goal_state.velocity[2]

        trans_control = np.array([x, y, z])

        # Determine forward direction
        if forward is None:
            # Use goal state quaternion to determine forward direction
            q_goal = GeoQuaternion(
                goal_state.quaternion[0],
                goal_state.quaternion[1],
                goal_state.quaternion[2],
                goal_state.quaternion[3]
            )
            R_goal = q_goal.getRotationMatrix()
            forward = R_goal[:, 1]  # Y-axis is typically forward
        else:
            forward = forward / np.linalg.norm(forward)  # Normalize

        # Calculate angular errors
        e_R, e_w, thrust = self.update_angular_error(trans_control, forward)

        # Attitude control (angular velocity control)
        wx = -self.kR * e_R[0] - self.kw * e_w[0] + self.goal_state.omega[0]
        wy = -self.kR * e_R[1] - self.kw * e_w[1] + self.goal_state.omega[1]
        wz = -self.kR * e_R[2] - self.kw * e_w[2] + self.goal_state.omega[2]

        return ControlCommand(thrust, np.array([wx, wy, wz]))

    def track_position(self, current_state: SE3State, goal_position: np.ndarray,
                      forward: Optional[np.ndarray] = None) -> ControlCommand:
        """Convenience method for position-only tracking.

        Args:
            current_state: Current quadrotor state
            goal_position: Desired position [x, y, z]
            forward: Desired forward direction

        Returns:
            Control command
        """
        # Create goal state (hovering at goal position)
        goal_state = SE3State(
            position=goal_position,
            velocity=np.zeros(3),
            quaternion=np.array([0.0, 0.0, 0.0, 1.0]),  # Identity quaternion
            omega=np.zeros(3)
        )

        return self.control_update(current_state, goal_state, 0.0, forward)


def create_default_se3_controller() -> SE3Controller:
    """Create SE3 controller with default parameters.

    Returns:
        SE3Controller instance with default quadrotor parameters
    """
    from .physics import QUAD_PARAMS
    return SE3Controller(QUAD_PARAMS)


def state_from_vector(state_vector: np.ndarray) -> SE3State:
    """Create SE3State from state vector.

    Args:
        state_vector: [pos(3), quat_wxyz(4), vel(3), omega(3)]

    Returns:
        SE3State instance
    """
    assert len(state_vector) == 13

    position = state_vector[0:3]
    quaternion_wxyz = state_vector[3:7]
    velocity = state_vector[7:10]
    omega = state_vector[10:13]

    # Convert quaternion from wxyz to xyzw
    quaternion_xyzw = np.array([
        quaternion_wxyz[1], quaternion_wxyz[2],
        quaternion_wxyz[3], quaternion_wxyz[0]
    ])

    return SE3State(position, velocity, quaternion_xyzw, omega)