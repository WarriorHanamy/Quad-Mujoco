# 20250220 Wakkk
# Quadrotor SE3 Control Demo
import mujoco 
import mujoco.viewer as viewer 
import numpy as np
from nmpc_controller import NMPC_Controller

# 新建NMPC控制器
controller = NMPC_Controller()

gravity = 9.8066        # 重力加速度 单位m/s^2
mass = 0.033            # 飞行器质量 单位kg
Ct = 3.25e-4            # 电机推力系数 (N/krpm^2)
Cd = 7.9379e-6          # 电机反扭系数 (Nm/krpm^2)

arm_length = 0.065/2.0  # 电机力臂长度 单位m
max_thrust = 0.1573     # 单个电机最大推力 单位N (电机最大转速22krpm)
max_torque = 3.842e-03  # 单个电机最大扭矩 单位Nm (电机最大转速22krpm)

# 仿真周期 100Hz 10ms 0.01s
dt = 0.01

# 根据电机转速计算电机推力
def calc_motor_force(krpm):
    global Ct
    return Ct * krpm**2

# 根据电机转速计算电机归一化输入
def calc_motor_input(krpm):
    if krpm > 22:
        krpm = 22
    elif krpm < 0:
        krpm = 0
    _force = calc_motor_force(krpm)
    _input = _force / max_thrust
    if _input > 1:
        _input = 1
    elif _input < 0:
        _input = 0
    return _input

# 加载模型回调函数
def load_callback(m=None, d=None):
    mujoco.set_mjcb_control(None)
    m = mujoco.MjModel.from_xml_path('./crazyfile/scene.xml')
    d = mujoco.MjData(m)
    if m is not None:
        mujoco.set_mjcb_control(lambda m, d: control_callback(m, d))  # 设置控制回调函数
    return m, d

# 根据四元数计算旋转矩阵
def rotation_matrix(q0, q1, q2, q3):
    _row0 = np.array([1-2*(q2**2)-2*(q3**2), 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)])
    _row1 = np.array([2*(q1*q2+q0*q3), 1-2*(q1**2)-2*(q3**2), 2*(q2*q3-q0*q1)])
    _row2 = np.array([2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 1-2*(q1**2)-2*(q2**2)])
    return np.vstack((_row0, _row1, _row2))

log_count = 0
def control_callback(m, d):
    global log_count, gravity, mass, controller
    _pos = d.qpos
    _vel = d.qvel
    _sensor_data = d.sensordata
    gyro_x = _sensor_data[0]
    gyro_y = _sensor_data[1]
    gyro_z = _sensor_data[2]
    acc_x = _sensor_data[3]
    acc_y = _sensor_data[4]
    acc_z = _sensor_data[5]
    quat_w = _sensor_data[6]
    quat_x = _sensor_data[7]
    quat_y = _sensor_data[8]
    quat_z = _sensor_data[9]
    quat = np.array([quat_x, quat_y, quat_z, quat_w])  # x y z w
    omega = np.array([gyro_x, gyro_y, gyro_z])         # 角速度
    # 构建当前状态
    current_state = np.array([_pos[0], _pos[1], _pos[2], quat[3], quat[0], quat[1], quat[2], _vel[0], _vel[1], _vel[2], omega[0], omega[1], omega[2]])
    # 位置控制模式 目标位点
    goal_position = np.array([0.0, 0.0, 0.5])

    # NMPC Update
    _dt, _control = controller.nmpc_position_control(current_state, goal_position)
    d.actuator('motor1').ctrl[0] = calc_motor_input(_control[0])
    d.actuator('motor2').ctrl[0] = calc_motor_input(_control[1])
    d.actuator('motor3').ctrl[0] = calc_motor_input(_control[2])
    d.actuator('motor4').ctrl[0] = calc_motor_input(_control[3])

    log_count += 1
    if log_count >= 50:
        log_count = 0
        # 这里输出log

if __name__ == '__main__':
    viewer.launch(loader=load_callback)
