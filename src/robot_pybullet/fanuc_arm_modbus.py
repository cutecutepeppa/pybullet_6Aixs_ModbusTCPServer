import math, time
import pybullet as p
import pybullet_data
import Modbus_Server_framework as mb

# === 建立物理環境 ===
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# 載入 Fanuc LRMate200iD 機械手臂 
robot = p.loadURDF(r"C:\Users\huiting\PY\rob\fanuc_lrmate200id_support\urdf\lrmate200id.urdf",useFixedBase=True)
#改變手臂顏色
p.changeVisualShape(robot,-1, rgbaColor=[0.55, 0.55, 0.55, 1])  # base_link 改紅色
p.changeVisualShape(robot, 0, rgbaColor=[0.6, 0.6, 0.6, 1])  # link 0 → 藍色
p.changeVisualShape(robot, 1, rgbaColor=[0.7, 0.7, 0.7, 1])  # link 1 → 藍色
p.changeVisualShape(robot, 2, rgbaColor=[0.8, 0.8, 0.8, 1])  # link 2 → 藍色
# === 啟動 Modbus ===
mb.start()

# 建立一個 list 來存放 GUI 的滑桿 (sliders)
slider_map = {}
# 逐一處理機械手臂的每個關節
for j in [3, 4, 5]:
    name = p.getJointInfo(robot, j)[1].decode("utf-8")
    slider_map[j] = p.addUserDebugParameter(f"{name} (manual)", -3.14, 3.14, 0.0)


while p.isConnected():
    
    val1, val2, val3 = mb.get_latest(modes=("dint","dint","float"))

    #沒有多餘的馬達所以軸三等於虛擬軸
    angle_deg_3 = val3
    # 1) 取餘數到單圈 0..65535
    cnt_mod_1 = val1 % 65536
    cnt_mod_2 = val2 % 65536

    # 2) 轉角度（度）
    # 將單圈 0..65535 映到 [-180, +180) 度
    angle_deg_1 = ((cnt_mod_1 / 65536.0) * 360.0)
    if angle_deg_1 >= 180.0:
        angle_deg_1 -= 360.0

    angle_deg_2 = ((cnt_mod_2 / 65536.0) * 360.0)
    if angle_deg_2 >= 180.0:
        angle_deg_2 -= 360.0

    # 3) 角度限制
    # 軸1 -170° ~ +170°
    LOWER_DEG_1=-170.0
    UPPER_DEG_1= 170.0
    if angle_deg_1 < LOWER_DEG_1:
        angle_deg_1 = LOWER_DEG_1
    elif angle_deg_1 > UPPER_DEG_1:
        angle_deg_1 = UPPER_DEG_1
    # 軸2 -100° ~ +145°
    LOWER_DEG_2=-100.0
    UPPER_DEG_2= 145.0
    if angle_deg_2 < LOWER_DEG_2:
        angle_deg_2 = LOWER_DEG_2
    elif angle_deg_2 > UPPER_DEG_2:
        angle_deg_2 = UPPER_DEG_2
    # 軸3 -70° ~ +205°
    LOWER_DEG_3=-70.0
    UPPER_DEG_3= 205.0
    if angle_deg_3 < LOWER_DEG_3:
        angle_deg_3 = LOWER_DEG_3
    elif angle_deg_3 > UPPER_DEG_3:
        angle_deg_3 = UPPER_DEG_3

    # 4) 轉成 rad（這一步要每圈做，否則 targetPosition 不會更新）
    arm1_angle = math.radians(angle_deg_1)
    arm2_angle = math.radians(angle_deg_2)
    arm3_angle = math.radians(angle_deg_3)

    # 5) 下達角度控制
    p.setJointMotorControlArray(
    bodyUniqueId=robot,
    jointIndices=[0, 1, 2],   # Joint 0 = 第1軸, Joint 1 = 第2軸
    controlMode=p.POSITION_CONTROL,
    targetPositions=[arm1_angle, arm2_angle,arm3_angle],  # 每個軸對應的目標位置 (rad)
    forces=[200, 200, 200]  # 每個軸的最大力矩限制
    )
    # 觀察：印原始/單圈/角度
    print(f"arm1={angle_deg_1:7.2f}° {angle_deg_2:7.2f}°  {angle_deg_3:7.2f}°  ")

    # 來自 PLC 的三軸
    tgt0, tgt1, tgt2 = arm1_angle, arm2_angle, arm3_angle

    # 來自滑桿的三軸
    tgt3 = p.readUserDebugParameter(slider_map[3])
    tgt4 = p.readUserDebugParameter(slider_map[4])
    tgt5 = p.readUserDebugParameter(slider_map[5])

    p.setJointMotorControlArray(
        bodyUniqueId=robot,
        jointIndices=[0,1,2,3,4,5],
        controlMode=p.POSITION_CONTROL,
        targetPositions=[tgt0, tgt1, tgt2, tgt3, tgt4, tgt5],
        forces=[200]*6
    )

    p.stepSimulation()
    time.sleep(1.0/240.0)

