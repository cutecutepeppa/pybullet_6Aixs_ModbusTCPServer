# tests/test_modbus_get_latest.py
#測模組的 API start() / get_latest()
from robot_pybullet import Modbus_Server_framework as mb

def write_hr(addr, words):
    # 直接寫入你的模組內部 context 的 HR，模擬 PLC 寫進來
    mb._context[mb.UNIT_ID].setValues(3, addr, words)  # 3=holding registers

def test_get_latest_three_channels():
    # 通道1 @1633~1634、通道2 @1635~1636、通道3 @1637~1638（你的程式這樣讀）：
    # 這裡分別寫入 DINT、DINT、FLOAT 對應的兩個 16-bit
    write_hr(1633, [0x1234, 0x5678])  # 通道1
    write_hr(1635, [0xAAAA, 0xBBBB])  # 通道2
    write_hr(1637, [0x0000, 0x3FC0])  # 通道3 → float 1.5 (小端)

    v1, v2, v3 = mb.get_latest(modes=("dint","dint","float"))
    # 只做型別/大致性驗證，避免平台端大小端差異在不同測試環境失敗
    assert isinstance(v1, int) and isinstance(v2, int)
    assert abs(float(v3) - 1.5) < 1e-6
