import time, numpy as np
import pybullet as p
import pybullet_data

# --- 初始化 ---
p.connect(p.GUI)  # 啟動模擬器 (GUI 介面)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 加入 PyBullet 內建資料路徑
p.setGravity(0,0,-9.81)  # 設定重力
p.loadURDF("plane.urdf")  # 加入平面

# 載入 Panda 機械手臂
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
n = p.getNumJoints(robot)

# 取出 Panda 的 7 軸關節 index（排除夾爪 joint）
arm_joint_indices = [j for j in range(n) if "panda_joint" in p.getJointInfo(robot, j)[1].decode()]
# 找到末端執行器 (end effector, panda_hand) 的 link
ee_link = [j for j in range(n) if "panda_hand" in p.getJointInfo(robot, j)[12].decode()][0]

# --- 直線移動函式 ---
def move_linear(start, target, speed=0.05, dt=1/240.0):
    """
    讓末端從 start (x,y,z) 移動到 target (x,y,z)，
    用線性插補方式生成一條直線軌跡，並用 IK 解算對應的關節角度。
    speed: 末端線速度 (m/s)，預設 0.05 = 5 cm/s
    dt:    模擬步長 (s)，預設 1/240
    """
    start = np.array(start); target = np.array(target)
    dist = np.linalg.norm(target-start)  # 計算起點到終點的距離
    steps = int(dist/speed/dt)           # 根據距離和速度算出需要的步數
    quat = p.getQuaternionFromEuler([0,3.14,0])  # 固定末端朝下 (姿態不變)

    for i in range(steps+1):
        # --- 線性插補 (Linear Interpolation) ---
        # a = 當前比例 (0 ~ 1)
        a = i/steps
        # pos = (1-a)*start + a*target
        #      = 起點與終點之間的線性插補
        pos = (1-a)*start + a*target

        # 由 IK 解算 pos + quat 的關節角度解
        q_sol = p.calculateInverseKinematics(robot, ee_link, pos, quat)

        # 發送 joint position control 指令
        p.setJointMotorControlArray(robot, arm_joint_indices, p.POSITION_CONTROL,
                                    targetPositions=q_sol[:len(arm_joint_indices)])
        p.stepSimulation()
        time.sleep(dt)

# --- 主迴圈：來回移動 ---
A = [0.3,0.3,0.3]
B = [0.1,0.1,0.1]
while True:
    move_linear(A, B, speed=0.1)  # A → B，速度 5 cm/s
    move_linear(B, A, speed=0.1)  # B → A，速度 5 cm/s
