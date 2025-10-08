"""Launch the MuJoCo viewer with a basic position-holding controller."""

import mujoco
import mujoco.viewer as viewer
import numpy as np

from quad_mujoco import QUAD_PARAMS, SimpleQuadrotorController


controller = SimpleQuadrotorController(QUAD_PARAMS)
goal_position = np.array([0.0, 0.0, 0.5])


def load_callback(model=None, data=None):
    mujoco.set_mjcb_control(None)
    model = mujoco.MjModel.from_xml_path("./crazyfile/scene.xml")
    data = mujoco.MjData(model)
    if model is not None:
        mujoco.set_mjcb_control(control_callback)
    return model, data


def control_callback(model, data):
    position = np.array(data.qpos[0:3])
    quaternion_wxyz = np.array(data.qpos[3:7])
    velocity = np.array(data.qvel[0:3])
    omega = np.array(data.qvel[3:6])

    state_vector = np.concatenate((position, quaternion_wxyz, velocity, omega))
    control = controller.track_position(state_vector, goal_position)

    data.actuator("motor1").ctrl[0] = control[0]
    data.actuator("motor2").ctrl[0] = control[1]
    data.actuator("motor3").ctrl[0] = control[2]
    data.actuator("motor4").ctrl[0] = control[3]


if __name__ == "__main__":
    viewer.launch(loader=load_callback)
