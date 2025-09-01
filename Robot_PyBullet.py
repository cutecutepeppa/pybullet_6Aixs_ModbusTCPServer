# pip install pybullet
import pybullet as p, pybullet_data, time
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf"); p.setGravity(0,0,-9.8)
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# 讓關節慢慢轉到指定角度
targets = [0, -0.6, 0, -2.0, 0, 1.8, 0.8]   # 7-DOF
for step in range(600):
    for j, q in enumerate(targets):
        p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=q, force=200)
    p.stepSimulation(); time.sleep(1/240)

