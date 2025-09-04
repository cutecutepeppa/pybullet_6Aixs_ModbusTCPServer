import pybullet as p, pybullet_data, time
import numpy as np

# --- 初始化 ---
cid = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
p.loadURDF("plane.urdf")

robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
n = p.getNumJoints(robot)

# 取出手臂關節 index
arm_joint_indices = [j for j in range(n) if "panda_joint" in p.getJointInfo(robot, j)[1].decode()]
ee_link = [j for j in range(n) if "panda_hand" in p.getJointInfo(robot, j)[12].decode()][0]

# --- 直線移動函式 ---
def move_linear(start, target, speed=0.10, dt=1/240.0):
    start = np.array(start, dtype=float)
    target = np.array(target, dtype=float)
    dist = np.linalg.norm(target - start)
    t_total = dist / speed
    steps = int(t_total / dt)

    for i in range(steps+1):
        alpha = i / steps
        pos = (1-alpha)*start + alpha*target
        orn = p.getQuaternionFromEuler([0, 3.14, 0])  # 姿態隨便給個朝下

        q_sol = p.calculateInverseKinematics(robot, ee_link, pos, orn)

        p.setJointMotorControlArray(
            robot,
            arm_joint_indices,
            p.POSITION_CONTROL,
            targetPositions=[q_sol[j] for j in range(len(arm_joint_indices))]
        )
        p.stepSimulation()
        time.sleep(dt)

# --- Demo ---
move_linear([1,0,0], [71,50,100], speed=1.5)
print("Done")
while True:
    p.stepSimulation()
    time.sleep(1/240)
