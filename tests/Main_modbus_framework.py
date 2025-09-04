# -*- coding: utf-8 -*-
import time
import Modbus_Server_framework as mb

if __name__ == "__main__":
    mb.start()

    print("主程式啟動，開始讀取 HR1633+1634 ...")
    while True:
        vals = mb.get_latest(mode="dint")        
        if vals:
            #v1, v2, v3 = vals 多個就這樣寫
            v1 = vals
            #print(f"Value1={v1}, Value2={v2}, Value3={v3}")
            print(f"Value1={v1}")        
        time.sleep(0.05)
