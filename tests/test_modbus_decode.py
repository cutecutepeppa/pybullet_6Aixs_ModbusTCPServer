# tests/test_modbus_decode.py
#測 Modbus 解碼
import math
from robot_pybullet import Modbus_Server_framework as mb

def test_decode_words_dint_le():
    # 低位字在前(low), 高位字在後(high)，對應小端
    low, high = 0xAAAA, 0x5555
    v = mb.decode_words(low, high, mode="dint", big_endian=False)
    # 驗值：用相同邏輯再編回來比對
    low2 = v & 0xFFFF
    high2 = (v >> 16) & 0xFFFF
    assert (low2, high2) == (low, high)

def test_decode_words_float_le():
    # 給一個已知 float (1.5) 的 16-bit 拆解，確認還原正確
    # 1.5 的 IEEE-754 十六進位在小端 16-bit 序列是 0x0000, 0x3FC0
    assert abs(mb.decode_words(0x0000, 0x3FC0, mode="float", big_endian=False) - 1.5) < 1e-6
