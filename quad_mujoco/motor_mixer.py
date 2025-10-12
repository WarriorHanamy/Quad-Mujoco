"""Advanced motor mixer for quadrotor control.

This module implements an intelligent motor mixing algorithm that handles
thrust and torque saturation, optimizing motor performance.
Based on the Quadrotor_SE3_Control repository implementation.
"""

import numpy as np
from typing import Tuple

from .physics import QuadrotorPhysicalParams


class MotorMixer:
    """Advanced motor mixer with saturation handling.

    This mixer intelligently distributes thrust and torques among motors
    while handling saturation constraints optimally.
    """

    def __init__(self, params: QuadrotorPhysicalParams):
        """Initialize motor mixer with physical parameters.

        Args:
            params: Quadrotor physical parameters
        """
        self.params = params

        # Power allocation matrix (forward)
        # Maps motor speeds to [total_thrust, Mx, My, Mz]
        self.mat = np.array([
            [params.thrust_coeff, params.thrust_coeff, params.thrust_coeff, params.thrust_coeff],
            [params.thrust_coeff * params.arm_length, -params.thrust_coeff * params.arm_length,
             -params.thrust_coeff * params.arm_length, params.thrust_coeff * params.arm_length],
            [-params.thrust_coeff * params.arm_length, -params.thrust_coeff * params.arm_length,
             params.thrust_coeff * params.arm_length, params.thrust_coeff * params.arm_length],
            [-params.drag_coeff, params.drag_coeff, -params.drag_coeff, params.drag_coeff]
        ])

        # Power allocation matrix (inverse)
        # Maps [total_thrust, Mx, My, Mz] to motor speeds squared
        self.inv_mat = np.linalg.inv(self.mat)

    def calculate(self, thrust: float, mx: float, my: float, mz: float) -> np.ndarray:
        """Calculate motor speeds for desired thrust and torques.

        This function implements an advanced mixing algorithm that:
        1. First distributes X, Y torques with priority
        2. Uses remaining capacity for Z torque if available
        3. Handles saturation constraints intelligently

        Args:
            thrust: Total thrust command (N)
            mx: Roll torque command (Nm)
            my: Pitch torque command (Nm)
            mz: Yaw torque command (Nm)

        Returns:
            Motor speeds in krpm
        """
        # First pass: handle X and Y torques only
        Mx, My = mx, my
        Mz = 0.0  # Initially no yaw torque

        control_input = np.array([thrust, Mx, My, Mz])
        motor_speed_squ = self.inv_mat @ control_input

        # Check for saturation
        max_value = np.max(motor_speed_squ)
        min_value = np.min(motor_speed_squ)
        ref_value = np.sum(motor_speed_squ) / 4.0  # Reference speed (no torque)

        # Calculate trim scales if saturation detected
        max_trim_scale = 1.0
        min_trim_scale = 1.0

        if max_value > self.params.max_rotor_speed**2:
            # Max speed saturation
            max_trim_scale = (self.params.max_rotor_speed**2 - ref_value) / (max_value - ref_value)

        if min_value < 0:
            # Negative speed saturation
            min_trim_scale = ref_value / (ref_value - min_value)

        scale = min(max_trim_scale, min_trim_scale)

        # Apply scaling to X, Y torques if needed
        Mx = Mx * scale
        My = My * scale

        # Recalculate motor speeds with scaled torques
        control_input = np.array([thrust, Mx, My, Mz])
        motor_speed_squ = self.inv_mat @ control_input

        if scale < 1.0:
            # Saturation occurred, no capacity for Z torque
            motor_speed_squ = np.abs(motor_speed_squ)
            return np.sqrt(motor_speed_squ)
        else:
            # Still have capacity, can add Z torque
            Mz = mz
            control_input_with_z = np.array([thrust, Mx, My, Mz])
            motor_speed_squ_with_z = self.inv_mat @ control_input_with_z

            # Check if Z torque causes saturation
            max_value_z = np.max(motor_speed_squ_with_z)
            min_value_z = np.min(motor_speed_squ_with_z)
            max_index = np.argmax(motor_speed_squ_with_z)
            min_index = np.argmin(motor_speed_squ_with_z)

            max_trim_scale_z = 1.0
            min_trim_scale_z = 1.0

            if max_value_z > self.params.max_rotor_speed**2:
                max_trim_scale_z = (self.params.max_rotor_speed**2 - motor_speed_squ[max_index]) / (max_value_z - motor_speed_squ[max_index])

            if min_value_z < 0:
                min_trim_scale_z = motor_speed_squ[min_index] / (motor_speed_squ[min_index] - min_value_z)

            scale_z = min(max_trim_scale_z, min_trim_scale_z)

            # Apply scaling to Z torque
            Mz = Mz * scale_z

            # Final motor speed calculation
            control_input_final = np.array([thrust, Mx, My, Mz])
            motor_speed_squ_final = self.inv_mat @ control_input_final

            motor_speed_squ = np.abs(motor_speed_squ_final)
            return np.sqrt(motor_speed_squ)

    def calculate_normalized(self, thrust: float, mx: float, my: float, mz: float) -> np.ndarray:
        """Calculate normalized motor commands (0-1).

        Args:
            thrust: Total thrust command (N)
            mx: Roll torque command (Nm)
            my: Pitch torque command (Nm)
            mz: Yaw torque command (Nm)

        Returns:
            Normalized motor commands in range [0, 1]
        """
        speeds = self.calculate(thrust, mx, my, mz)
        # Convert speeds to thrust and then normalize
        thrusts = self.params.thrust_coeff * speeds**2
        return np.clip(thrusts / self.params.max_thrust_per_motor, 0.0, 1.0)

    def get_allocation_matrix(self) -> np.ndarray:
        """Get the forward allocation matrix.

        Returns:
            4x4 allocation matrix mapping motor speeds to wrench
        """
        return self.mat.copy()

    def get_inverse_allocation_matrix(self) -> np.ndarray:
        """Get the inverse allocation matrix.

        Returns:
            4x4 inverse allocation matrix mapping wrench to motor speeds
        """
        return self.inv_mat.copy()


def create_default_mixer() -> MotorMixer:
    """Create a motor mixer with default parameters.

    Returns:
        MotorMixer instance with default quadrotor parameters
    """
    from .physics import QUAD_PARAMS
    return MotorMixer(QUAD_PARAMS)