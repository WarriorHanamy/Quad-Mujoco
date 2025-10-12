"""Compare performance between SimpleQuadrotorController and SE3Controller."""

import mujoco
import numpy as np
import time
from typing import List, Dict

from quad_mujoco import (
    QUAD_PARAMS,
    SimpleQuadrotorController,
    SE3Controller,
    SE3State,
    MotorMixer,
    state_from_vector,
    create_default_mixer,
)


def run_simulation(controller_type: str, duration: float = 10.0, trajectory_type: str = "step") -> Dict:
    """Run simulation with specified controller.

    Args:
        controller_type: "simple" or "se3"
        duration: Simulation duration in seconds
        trajectory_type: "step", "circle", or "hover"

    Returns:
        Dictionary with simulation results
    """
    # Initialize simulation
    model = mujoco.MjModel.from_xml_path("./crazyfile/scene.xml")
    data = mujoco.MjData(model)

    # Initialize controller
    if controller_type == "simple":
        controller = SimpleQuadrotorController(QUAD_PARAMS)
        controller.kp_pos = np.array([3.0, 3.0, 6.0])
        controller.kd_pos = np.array([1.5, 1.5, 3.0])
    elif controller_type == "se3":
        controller = create_default_se3_controller()
        controller.set_gains(kx=1.0, kv=0.6, kR=8.0, kw=1.5)
        motor_mixer = create_default_mixer()
    else:
        raise ValueError("Invalid controller type")

    # Simulation parameters
    dt = 0.001  # 1ms timestep
    steps = int(duration / dt)

    # Trajectory parameters
    if trajectory_type == "step":
        target_pos = np.array([0.5, 0.3, 0.6])
        target_heading = np.array([1.0, 0.0, 0.0])
    elif trajectory_type == "circle":
        radius = 0.5
        height = 0.4
        speed = 0.2
    elif trajectory_type == "hover":
        target_pos = np.array([0.0, 0.0, 0.5])
        target_heading = np.array([1.0, 0.0, 0.0])
    else:
        raise ValueError("Invalid trajectory type")

    # Data logging
    positions = []
    targets = []
    errors = []
    times = []

    print(f"Running {controller_type} controller with {trajectory_type} trajectory...")

    # Run simulation
    start_time = time.time()
    for step in range(steps):
        current_time = step * dt

        # Get current state
        position = np.array(data.qpos[0:3])
        quaternion_wxyz = np.array(data.qpos[3:7])
        velocity = np.array(data.qvel[0:3])
        omega = np.array(data.qvel[3:6])

        # Generate trajectory
        if trajectory_type == "circle":
            angle = 2 * np.pi * speed * current_time
            target_pos = np.array([
                radius * np.cos(angle),
                radius * np.sin(angle),
                height
            ])
            target_heading = np.array([-np.sin(angle), np.cos(angle), 0.0])

        # Control command
        if controller_type == "simple":
            state_vector = np.concatenate((position, quaternion_wxyz, velocity, omega))
            motor_commands = controller.track_position(state_vector, target_pos)
        else:  # se3
            current_state = SE3State(
                position=position,
                velocity=velocity,
                quaternion=np.array([quaternion_wxyz[1], quaternion_wxyz[2], quaternion_wxyz[3], quaternion_wxyz[0]]),
                omega=omega
            )
            control_command = controller.track_position(current_state, target_pos, target_heading)
            total_thrust = control_command.thrust * QUAD_PARAMS.gravity * QUAD_PARAMS.mass
            total_torque = control_command.angular * 0.001
            motor_commands = motor_mixer.calculate_normalized(total_thrust, total_torque[0], total_torque[1], total_torque[2])

        # Apply motor commands
        data.actuator("motor1").ctrl[0] = motor_commands[0]
        data.actuator("motor2").ctrl[0] = motor_commands[1]
        data.actuator("motor3").ctrl[0] = motor_commands[2]
        data.actuator("motor4").ctrl[0] = motor_commands[3]

        # Step simulation
        mujoco.mj_step(model, data)

        # Log data
        if step % 100 == 0:  # Log every 100 steps
            positions.append(position.copy())
            targets.append(target_pos.copy())
            errors.append(np.linalg.norm(position - target_pos))
            times.append(current_time)

    end_time = time.time()

    return {
        "positions": np.array(positions),
        "targets": np.array(targets),
        "errors": np.array(errors),
        "times": np.array(times),
        "simulation_time": end_time - start_time,
        "final_error": errors[-1] if errors else float('inf'),
        "max_error": np.max(errors) if errors else float('inf'),
        "mean_error": np.mean(errors) if errors else float('inf')
    }


def compare_controllers():
    """Compare performance of both controllers."""
    print("Controller Performance Comparison")
    print("=" * 50)

    trajectories = ["hover", "step", "circle"]
    results = {}

    for traj in trajectories:
        print(f"\nTesting {traj} trajectory:")
        print("-" * 30)

        # Test simple controller
        simple_results = run_simulation("simple", duration=5.0, trajectory_type=traj)
        print(f"Simple Controller - Final error: {simple_results['final_error']:.3f}m, "
              f"Mean error: {simple_results['mean_error']:.3f}m, "
              f"Max error: {simple_results['max_error']:.3f}m")

        # Test SE3 controller
        se3_results = run_simulation("se3", duration=5.0, trajectory_type=traj)
        print(f"SE3 Controller   - Final error: {se3_results['final_error']:.3f}m, "
              f"Mean error: {se3_results['mean_error']:.3f}m, "
              f"Max error: {se3_results['max_error']:.3f}m")

        results[traj] = {
            "simple": simple_results,
            "se3": se3_results
        }

        # Calculate improvement
        improvement = (simple_results['mean_error'] - se3_results['mean_error']) / simple_results['mean_error'] * 100
        print(f"SE3 improvement: {improvement:.1f}%")

    # Summary
    print("\n" + "=" * 50)
    print("Summary:")
    print("-" * 30)
    for traj in trajectories:
        simple_err = results[traj]["simple"]["mean_error"]
        se3_err = results[traj]["se3"]["mean_error"]
        improvement = (simple_err - se3_err) / simple_err * 100
        print(f"{traj.capitalize():8s}: SE3 {improvement:+5.1f}% vs Simple")


if __name__ == "__main__":
    compare_controllers()