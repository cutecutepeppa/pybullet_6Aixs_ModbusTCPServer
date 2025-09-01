import struct
from pymodbus.server.sync import StartTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock
from pymodbus.device import ModbusDeviceIdentification
import logging

# 關閉 pymodbus 日誌
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.ERROR)

# === 初始化 Modbus 資料區（Holding Registers） ===
# HR（Holding Register）從位址 1633 開始，共建立 100 個 16-bit 暫存器（模擬 HR 1633 ~ 1732）
store = ModbusSlaveContext(
    hr=ModbusSequentialDataBlock(1633, [0]*100)
)

# 建立 Modbus Server Context，slave ID 為 255，支援多 slave 模式（single=False）
context = ModbusServerContext(slaves={255: store}, single=False)

# === 將兩個 16-bit WORD 合併為 32-bit float ===
def decode_float_pair(low_word, high_word):
    word_bytes = struct.pack('<HH', low_word, high_word)  # 小端序打包成 4 bytes
    return struct.unpack('<f', word_bytes)[0]             # 解析成 float（IEEE 754 格式）

# 例：讀取 3 組 float（RPM、Value、Vol）
def read_floats():
    regs = context[255].getValues(6, 1633, count=6)
    rpm = decode_float_pair(regs[0], regs[1])      # HR 1633 + 1634
    value = decode_float_pair(regs[2], regs[3])    # HR 1635 + 1636
    vol = decode_float_pair(regs[4], regs[5])      # HR 1637 + 1638
    return rpm, value, vol

if __name__ == "__main__":
    # 啟動 Modbus TCP Slave Server，監聽 502 埠
    print("啟動 Modbus TCP Slave (ID=255)，HR 1633~1732 可用於 PLC 通訊...")
    StartTcpServer(context, address=("0.0.0.0", 502))
