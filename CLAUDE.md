# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a lightweight quadrotor simulation framework using MuJoCo for physics and visualization, with two different control implementations:
- **SimpleQuadrotorController**: A cascaded PID-style controller
- **SE3Controller**: A geometric SE(3) manifold controller for more rigorous control

The project uses `uv` for dependency management and `just` for common tasks.

## Common Commands

### Environment Setup
```bash
uv sync                     # Install dependencies
```

### Running Simulations
```bash
# Basic position-holding controller (recommended for quick testing)
uv run python view_entry.py
just view                   # Equivalent command if just is installed

# SE3 geometric controller (advanced)
uv run python main_se3.py
just main                   # Equivalent command if just is installed

# Performance comparison between controllers
uv run python compare_controllers.py

# Simple MuJoCo viewer without controller
uv run python main.py
```

### Development Workflow
```bash
just work                   # Load tmux development session (requires tmuxp)
just k                      # Kill tmux session
```

## Architecture

### Core Components

**quad_mujoco/** package contains the main simulation logic:

- `physics.py`: Physical parameters (`QUAD_PARAMS`) and utility functions for quadrotor dynamics
- `controller.py`: `SimpleQuadrotorController` - cascaded position + attitude controller
- `se3_controller.py`: `SE3Controller` - geometric SE(3) controller with advanced state handling
- `motor_mixer.py`: `MotorMixer` class for converting thrust/torque commands to motor outputs
- `geometry_se3.py`: SE(3) geometric utilities including quaternion operations and hat/vee maps
- `__init__.py`: Public API exports

**Entry Points**:

- `view_entry.py`: Lightweight viewer with basic position controller
- `main_se3.py`: Full SE3 controller implementation with trajectory tracking
- `main.py`: Simple MuJoCo viewer without control
- `compare_controllers.py`: Performance benchmarking tool

**MuJoCo Model**:
- `crazyfile/scene.xml`: Quadrotor model with 4 motor actuators expecting normalized thrust commands [0,1]

### Key Data Structures

**Physical Parameters** (`QuadrotorPhysicalParams` in physics.py):
- Mass: 0.033 kg
- Inertia: (1.395e-5, 1.395e-5, 2.173e-5) kg·m²
- Max rotor speed: 22.0 krpm
- Thrust/drag coefficients defined

**State Representations**:
- `QuadrotorState`: position, quaternion_wxyz, velocity, body_rates
- `SE3State`: position, velocity, quaternion_xyzw, omega (note: different quaternion order)

**Control Commands**:
- Simple controller returns normalized motor commands [0,1]
- SE3 controller returns `ControlCommand` (thrust in g units, angular torques)

### Controller Differences

**SimpleQuadrotorController**:
- Cascaded design with position → attitude → motor mapping
- Uses Euler angles for attitude representation
- Simpler to tune, good for basic hovering
- Gains: `kp_pos`, `kd_pos`, `kp_att`, `kd_att`

**SE3Controller**:
- Geometric control on SE(3) manifold
- Uses quaternions, globally asymptotically stable
- More complex but mathematically rigorous
- Gains: `kx`, `kv`, `kR`, `kw`
- Requires `MotorMixer` for actuator allocation

### Motor Mixer

The `MotorMixer` class handles:
- Converting total thrust (N) and torques (N·m) to individual motor commands
- Saturation constraints and allocation matrix calculations
- Output normalization to [0,1] range for MuJoCo actuators

### Simulation Loop Pattern

All viewers follow this pattern:
1. Load MuJoCo model from `./crazyfile/scene.xml`
2. Extract state: position (qpos[0:3]), quaternion (qpos[3:7]), velocity (qvel[0:3]), omega (qvel[3:6])
3. Compute control commands based on current state and goal
4. Apply commands to actuators (`data.actuator("motor1-4").ctrl[0]`)
5. MuJoCo handles physics integration automatically

## Physical Constants

Key constants from `QUAD_PARAMS`:
- Gravity: 9.8066 m/s²
- Mass: 0.033 kg
- Arm length: 0.0325 m
- Motor commands expected in [0,1] (normalized thrust)

## Development Notes

- Quaternion conventions differ between controllers (wxyz vs xyzw)
- SE3 controller requires torque scaling factor (typically 0.001)
- Both controllers can be tuned via gain parameters
- Motor saturation is handled automatically by MotorMixer
- Press `Esc` in MuJoCo viewer to exit