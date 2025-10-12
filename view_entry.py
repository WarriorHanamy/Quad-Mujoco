# 20250220 Wakkk
# Quadrotor SE3 Control Demo
import mujoco
import mujoco.viewer as viewer

# 加载模型回调函数
def load_callback(m=None, d=None):
    mujoco.set_mjcb_control(None)
    m = mujoco.MjModel.from_xml_path('./crazyfile/scene.xml')
    d = mujoco.MjData(m)
    return m, d

if __name__ == '__main__':
    viewer.launch(loader=load_callback)
