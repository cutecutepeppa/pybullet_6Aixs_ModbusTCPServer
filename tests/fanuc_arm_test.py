import pybullet as p
import pybullet_data
import time
import math
# --- 建立物理環境 ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# 載入平面 & 機械手臂 (改成你的完整路徑)
robot = p.loadURDF(
    r"C:\Users\huiting\PY\robot_Arm\fanuc-noetic-devel\fanuc_lrmate200id_support\urdf\lrmate200id.urdf",
    useFixedBase=True
)
p.loadURDF("plane.urdf")

# --- Joint 1 的限制 (用角度) ---
lower_limit_deg = 30
upper_limit_deg =  150

lower_limit = math.radians(lower_limit_deg)
upper_limit = math.radians(upper_limit_deg)
# --- 運動參數 ---
pos = lower_limit
speed_deg_per_sec = 85
direction = 1
# --- number轉 度/s ---
step = (speed_deg_per_sec * 3.14159 / 180) / 240
while p.isConnected():
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=0,
        controlMode=p.POSITION_CONTROL,
        targetPosition=pos,
        force=200
    )
    p.stepSimulation()
    time.sleep(1./240.)

    pos += direction * step
    if pos >= upper_limit or pos <= lower_limit:
        direction *= -1
