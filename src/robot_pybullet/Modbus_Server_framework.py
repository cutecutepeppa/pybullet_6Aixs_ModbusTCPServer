# -*- coding: utf-8 -*-
"""
modbus_reader.py
最小化 Modbus TCP Server 模組
- 建 HR 空間：1633 起，長度 100
- 提供 start() 開伺服器
- 提供 get_latest() 回傳解析後的 DINT 數值
"""

import struct
import threading
from pymodbus.server.sync import StartTcpServer
from pymodbus.datastore import (ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock)


# ---------------- 基本參數 ----------------
HOST = "0.0.0.0"   # ip客製化：Server 監聽 IP
PORT = 502        # port客製化：502 需權限；建議先用 5020
UNIT_ID = 255      # unit/slave id客製化：與 PLC 對應
START_ADDR = 1633  # HR 起始位址
HR_LENGTH  = 100   # HR 長度
BIG_ENDIAN = False # 大小端客製化：False=低字在前；True=高字在前
# -----------------------------------------

# 建 HR 區塊
hr_block = ModbusSequentialDataBlock(START_ADDR, [0]*HR_LENGTH)
_store = ModbusSlaveContext(hr=hr_block, zero_mode=False)
_context = ModbusServerContext(slaves={UNIT_ID: _store}, single=False)

_server_thread: threading.Thread = None
_server_started = False
_server_lock = threading.Lock()

# ---------------- 通用解碼函數 ----------------
def decode_words(low, high, mode="dint", big_endian=False):
    """
    兩個 16-bit word → DINT / UDINT / FLOAT
    mode: "dint" | "udint" | "float"
    """
    if big_endian:
        b = struct.pack(">HH", high & 0xFFFF, low & 0xFFFF)
    else:
        b = struct.pack("<HH", low & 0xFFFF, high & 0xFFFF)

    if mode == "dint":
        return struct.unpack(">i" if big_endian else "<i", b)[0]
    elif mode == "udint":
        return struct.unpack(">I" if big_endian else "<I", b)[0]
    elif mode == "float":
        return struct.unpack(">f" if big_endian else "<f", b)[0]
    else:
        raise ValueError("mode 必須是 'dint' | 'udint' | 'float'")

# ---------------- 對外 API ----------------
def start(host: str = HOST, port: int = PORT):
    """啟動 Modbus TCP Server（背景執行緒）"""
    global _server_thread, _server_started
    with _server_lock:
        if _server_started:
            return
        def _run():
            StartTcpServer(_context, address=(host, port))
        _server_thread = threading.Thread(target=_run, daemon=True)
        _server_thread.start()
        _server_started = True

def get_latest(modes=None):
    """
    讀取 HR → 回傳指定型別的值
    modes: tuple/list，指定每個通道的型別，例如 ("dint","dint","float")
    如果沒給，預設三個都是 "dint"
    """
    if modes is None:
        modes = ("dint", "dint", "dint")   # 預設值

    regs_1 = _context[UNIT_ID].getValues(3, 1633, count=2)
    regs_2 = _context[UNIT_ID].getValues(3, 1635, count=2)
    regs_3 = _context[UNIT_ID].getValues(3, 1637, count=2)

    val1 = decode_words(regs_1[0], regs_1[1], mode=modes[0], big_endian=BIG_ENDIAN)
    val2 = decode_words(regs_2[0], regs_2[1], mode=modes[1], big_endian=BIG_ENDIAN)
    val3 = decode_words(regs_3[0], regs_3[1], mode=modes[2], big_endian=BIG_ENDIAN)

    return (val1, val2, val3)