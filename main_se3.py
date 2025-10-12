"""Launch MuJoCo viewer with SE3 geometric position controller."""

import mujoco
import mujoco.viewer as viewer
import numpy as np

from quad_mujoco import (
    QUAD_PARAMS,
    SE3Controller,
    SE3State,
    MotorMixer,
    create_default_se3_controller,
    create_default_mixer,
    state_from_vector,
)


def get_hover_position(time: float) -> tuple[np.ndarray, np.ndarray]:
    """Get hover position and heading for position holding task.

    Args:
        time: Current simulation time

    Returns:
        Tuple of (target_position, target_heading)
    """
    # For position holding, always return the fixed goal position
    return goal_position, np.array([1.0, 0.0, 0.0])  # Point along X-axis


def load_callback(model=None, data=None):
    """Model loading callback for MuJoCo viewer."""
    mujoco.set_mjcb_control(None)
    model = mujoco.MjModel.from_xml_path("./crazyfile/scene.xml")
    data = mujoco.MjData(model)
    if model is not None:
        mujoco.set_mjcb_control(control_callback)
    return model, data


# Initialize controllers
se3_controller = create_default_se3_controller()
motor_mixer = create_default_mixer()

# Tune SE3 controller gains for better performance
se3_controller.set_gains(kx=0.8, kv=0.5, kR=8.0, kw=1.2)

# Control parameters
torque_scale = 0.001  # Scale factor for controller torque to actual torque (Nm)
goal_position = np.array([0.0, 0.0, 0.5])
use_trajectory = False  # Set to False for simple hover (position holding task)

log_count = 0


def control_callback(model, data):
    """Main control callback for SE3 controller."""
    global log_count, se3_controller, motor_mixer, torque_scale, goal_position, use_trajectory

    # Extract current state from MuJoCo
    position = np.array(data.qpos[0:3])
    quaternion_wxyz = np.array(data.qpos[3:7])
    velocity = np.array(data.qvel[0:3])
    omega = np.array(data.qvel[3:6])

    # Create current state for SE3 controller
    current_state = SE3State(
        position=position,
        velocity=velocity,
        quaternion=np.array([quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3], quaternion_wxyz[0]]),  # xyzw
        omega=omega
    )

    # Get target position and heading
    if use_trajectory:
        target_pos, target_heading = get_hover_position(data.time)
    else:
        target_pos = goal_position
        target_heading = np.array([1.0, 0.0, 0.0])  # Point along X-axis

    # Generate control command
    control_command = se3_controller.track_position(current_state, target_pos, target_heading)

    # Convert control command to motor commands
    # Thrust is in g units, convert to Newtons
    total_thrust = control_command.thrust * QUAD_PARAMS.gravity * QUAD_PARAMS.mass
    total_torque = control_command.angular * torque_scale

    # Use motor mixer to distribute thrust and torques
    motor_commands = motor_mixer.calculate_normalized(
        total_thrust, total_torque[0], total_torque[1], total_torque[2]
    )

    # Send commands to motors
    data.actuator("motor1").ctrl[0] = motor_commands[0]
    data.actuator("motor2").ctrl[0] = motor_commands[1]
    data.actuator("motor3").ctrl[0] = motor_commands[2]
    data.actuator("motor4").ctrl[0] = motor_commands[3]

    # Logging (every 500 steps)
    log_count += 1
    if log_count >= 500:
        log_count = 0
        print(f"Time: {data.time:.2f}s")
        print(f"Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        print(f"Target:   [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
        pos_error = np.linalg.norm(position - target_pos)
        print(f"Position error: {pos_error:.3f}m")
        print(f"Thrust: {control_command.thrust:.3f}g")
        print(f"Motors: [{motor_commands[0]:.2f}, {motor_commands[1]:.2f}, {motor_commands[2]:.2f}, {motor_commands[3]:.2f}]")
        print("-" * 50)


if __name__ == "__main__":
    print("SE3 Geometric Controller - Position Holding Task")
    print("=" * 50)
    print("Task:")
    print(f"- The quadrotor will hold position at: [{goal_position[0]:.1f}, {goal_position[1]:.1f}, {goal_position[2]:.1f}]")
    print("- SE3 controller provides geometric control on SE(3) manifold")
    print("- Advanced motor mixer handles saturation constraints")
    print("=" * 50)
    viewer.launch(loader=load_callback)