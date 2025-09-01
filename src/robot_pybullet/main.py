# -*- coding: utf-8 -*-
# 目標：Python 當 Modbus TCP Slave，接收 PLC 寫入的 AEP (HR 1633+1634, DINT)
#       計算 angle = (AEP % 65536) / 65536 * 360
#       把角度(轉為 rad)指定給 PyBullet 機械手臂第 0 軸

import struct, time, threading
import pybullet as p, pybullet_data
from pymodbus.server.sync import StartTcpServer
from pymodbus.datastore import (
    ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock
)

# ====== Modbus 參數 ======
HR_BASE   = 1600          # 提前建立一大片，避免超界
HR_SIZE   = 100           # 覆蓋 1600~1699，包含 1633/1634
SLAVE_ID  = 255           # 與你 PLC 設定一致
PORT      = 502           # 沒權限可改 5020，PLC 端也要改
ADDR_LOW  = 1633          # 低字在 1633，高字在 1634

# ====== 建立 Modbus Server ======
hr_block = ModbusSequentialDataBlock(HR_BASE, [0]*HR_SIZE)
store    = ModbusSlaveContext(hr=hr_block)
context  = ModbusServerContext(slaves={SLAVE_ID: store}, single=False)

def run_modbus():
    print(f"啟動 Modbus TCP Slave (unit={SLAVE_ID})，監聽 0.0.0.0:{PORT} ...")
    print("提醒：PLC（169.254.11.8）請把目標 IP 設為『你這台電腦的 IP』，埠號同上。")
    StartTcpServer(context, address=("0.0.0.0", PORT))

threading.Thread(target=run_modbus, daemon=True).start()

# ====== 輔助函數 ======
def decode_dint_le(low_word, high_word):
    """兩個 16-bit word (HR) → 32-bit 無號整數（小端序：LOW, HIGH）"""
    b = struct.pack("<HH", low_word & 0xFFFF, high_word & 0xFFFF)
    return struct.unpack("<I", b)[0]

def aep_to_deg(aep, ppr=65536):
    """AEP 轉角度 0~360"""
    return (aep % ppr) / ppr * 360.0

# ====== PyBullet 初始化 ======
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
p.setGravity(0, 0, -9.8)

# ====== 主迴圈 ======
try:
    last_print = 0.0
    while True:
        regs = context[SLAVE_ID].getValues(3, ADDR_LOW, count=2)  # 3=Holding
        if len(regs) >= 2:
            low, high = regs[0], regs[1]
            aep = decode_dint_le(low, high)
            deg = aep_to_deg(aep)
            rad = deg * 3.141592653589793 / 180.0

            # 控制 Panda 第0軸（panda_joint1）
            p.setJointMotorControl2(
                bodyUniqueId=robot,
                jointIndex=0,
                controlMode=p.POSITION_CONTROL,
                targetPosition=rad,
                force=200
            )

            # 每 0.3 秒印一次目前值（避免洗版）
            now = time.time()
            if now - last_print > 0.3:
                print(f"AEP={aep:10d} | LOW={low:5d} HIGH={high:5d} | {deg:7.2f}° -> {rad:8.4f} rad")
                last_print = now

        p.stepSimulation()
        time.sleep(1/240)

except KeyboardInterrupt:
    print("程式中斷，關閉連線...")
finally:
    p.disconnect()
