import pybullet as p
import pybullet_data
import time

# 1) 初始化
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# Panda 的末端執行器 link index（URDF 定義，Panda 通常是 11）
end_effector_index = 11

# 2) 初始位置 (x,y,z)
target_pos = [0.5, 0.0, 0.5]

while True:
    # 3) 用 IK 算出關節角度解
    joint_poses = p.calculateInverseKinematics(robot,
                                               end_effector_index,
                                               target_pos)
    # 4) 送出關節命令
    p.setJointMotorControlArray(robot,
                                jointIndices=list(range(7)),  # 前7軸
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=joint_poses[:7])

    # 5) 模擬一步
    p.stepSimulation()
    time.sleep(1/240)
