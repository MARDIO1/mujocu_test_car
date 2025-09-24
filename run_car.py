import mujoco
import mujoco.viewer
import os
import time
mjcf_file_path = os.path.join(os.path.dirname(__file__), 'car_all.xml')
model = mujoco.MjModel.from_xml_path(mjcf_file_path)
data = mujoco.MjData(model)

print("MuJoCo模型加载完成")
print("Launching viewer with custom side orthographic view...")

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 初始化 free camera
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)

    # 手动设置自由相机参数
    '''viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = [13, 0, 1.0]   # 场景中点，大概看中间（Z 调成 1 m 高度）
    viewer.cam.distance = 40              # 缩放，使目标和起点都进画面
    viewer.cam.azimuth = 90                # 90° 表示沿 Y 轴负方向→X 正方向看
    viewer.cam.elevation = 0               # 水平
    viewer.cam.orthographic = 1            # 正交投影'''

    # 检查地面是否存在
    ground_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ground")
    print(f"地面ID: {ground_id}")  # 应该输出 >=0
    # 2. 检查小车
    cart_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cart")
    print(f"小车ID: {cart_id}")
    # 检查小车位置
    cart_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cart")
    print(f"小车初始位置: {data.qpos[:3]}")  # 应该在地面上方

    # 2. 获取驱动器ID（通过名称 "cart_motor"）
    motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "cart_motor")
    print("驱动器ID：", motor_id)
    # 3. 设置控制信号（范围：-1 到 1）
    # 正值：小车向右移动，负值：小车向左移动
    #data.ctrl[motor_id] = 0.5  # 设置为50%功率向右移动

    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
        print(f"小车位置: {data.qpos[:3]}")  # 应该在地面上方
        