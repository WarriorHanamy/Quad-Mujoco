# 20250220 Wakkk
# Quadrotor SE3 Control Demo
import mujoco
import mujoco.viewer as viewer
import numpy as np

from quad_mujoco import QUAD_PARAMS, SimpleQuadrotorController


controller = SimpleQuadrotorController(QUAD_PARAMS)

goal_position = np.array([0.0, 0.0, 0.5])

# 加载模型回调函数
def load_callback(m=None, d=None):
    mujoco.set_mjcb_control(None)
    m = mujoco.MjModel.from_xml_path('./crazyfile/scene.xml')
    d = mujoco.MjData(m)
    if m is not None:
        mujoco.set_mjcb_control(lambda m, d: control_callback(m, d))  # 设置控制回调函数
    return m, d

log_count = 0
def control_callback(m, d):
    global log_count

    position = np.array(d.qpos[0:3])
    quaternion_wxyz = np.array(d.qpos[3:7])
    velocity = np.array(d.qvel[0:3])
    omega = np.array(d.qvel[3:6])

    state_vector = np.concatenate((position, quaternion_wxyz, velocity, omega))
    control = controller.track_position(state_vector, goal_position)

    d.actuator('motor1').ctrl[0] = control[0]
    d.actuator('motor2').ctrl[0] = control[1]
    d.actuator('motor3').ctrl[0] = control[2]
    d.actuator('motor4').ctrl[0] = control[3]

    log_count += 1
    if log_count >= 50:
        log_count = 0
        # 这里输出log

if __name__ == '__main__':
    viewer.launch(loader=load_callback)
