# Quad MuJoCo Demo

This repository contains a lightweight MuJoCo scene for a quadrotor together with a simple cascaded controller implemented only with NumPy. The original ACADOS-based NMPC formulation has been removed so the project can be managed purely with `pyproject.toml` + `uv` and standard scientific Python packages.

## Physical model

```python
from quad_mujoco import QUAD_PARAMS
print(QUAD_PARAMS)
```

Key constants:

- `gravity = 9.8066  # m/s^2`
- `mass = 0.033      # kg`
- `inertia = (1.395e-5, 1.395e-5, 2.173e-5)  # kg m^2`
- `thrust_coeff = 3.25e-4  # N / (krpm^2)`
- `drag_coeff = 7.9379e-6 # N·m / (krpm^2)`
- `arm_length = 0.0325    # m`
- `max_rotor_speed = 22.0 # krpm`

The actuator block in `crazyfile/scene.xml` expects normalised thrust commands in `[0, 1]`, which the controller produces directly.

## Running the demo

```bash
uv sync
uv run python main.py
```

Press `Esc` inside the viewer to exit.

For a light-weight run that uses the basic position-holding controller, execute:

```bash
uv run python view_entry.py
```

After installing `just`, you can simply call `just view`.

## Files

- `main.py` launches the MuJoCo viewer and feeds commands from the controller.
- `view_entry.py` opens the MuJoCo scene with the minimal position controller.
- `quad_mujoco/controller.py` contains the cascaded position + attitude controller.
- `quad_mujoco/physics.py` stores the physical description of the vehicle and helper math utilities.
- `crazyfile/scene.xml` is the MuJoCo model of the quadrotor.

Feel free to tweak the gains inside `SimpleQuadrotorController` to experiment with different flight behaviour.
