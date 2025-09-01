from pymodbus.server.sync import StartTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock
import time, threading

# ---------------------
HR_START = 1600     # 提前一點建立，避免超界
SLAVE_ID = 255
PORT = 502
# ---------------------

# 建立一塊足夠大的區塊，至少要包含 1633~1634
hr_block = ModbusSequentialDataBlock(HR_START, [0]*100)  # 1600~1699
store = ModbusSlaveContext(hr=hr_block)
context = ModbusServerContext(slaves={SLAVE_ID: store}, single=False)

def poll():
    last_val = None
    while True:
        rr = context[SLAVE_ID].getValues(3, 1633, count=2)  # 直接讀 1633~1634
        if len(rr) >= 2:
            low, high = rr[0], rr[1]
            val = (high << 16) | (low & 0xFFFF)
            if val != last_val:
                last_val = val
                print(f"Got: LOW={low}, HIGH={high}, UINT32={val}")
        time.sleep(0.1)

if __name__ == "__main__":
    t = threading.Thread(target=poll, daemon=True)
    t.start()
    StartTcpServer(context, address=("0.0.0.0", PORT))
