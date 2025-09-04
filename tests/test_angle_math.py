# tests/test_angle_math.py
#測角度換算與夾限
import math

def count_to_deg_symmetric(x):
    # 映射 0..65535 → [-180, 180)
    deg = ( (x % 65536) / 65536.0 ) * 360.0
    return deg - 360.0 if deg >= 180.0 else deg

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def test_count_to_deg_wraparound():
    assert count_to_deg_symmetric(0) == 0.0
    assert count_to_deg_symmetric(32768) == -180.0  # 180 映為 -180 邊界
    # 任意值檢查是否落在 [-180,180)
    for k in (100, 12345, 65535):
        assert -180.0 <= count_to_deg_symmetric(k) < 180.0

def test_joint_limits_and_radians():
    # 關節1：-170~170；關節2：-100~145；關節3：-70~205（度）
    # 隨便挑幾組邊界測
    j1 = clamp(count_to_deg_symmetric(40000), -170.0, 170.0)
    j2 = clamp(count_to_deg_symmetric(50000), -100.0, 145.0)
    j3 = clamp(120.0, -70.0, 205.0)  # 軸3 來源是 float 角度
    # 轉 rad
    r1, r2, r3 = map(math.radians, (j1, j2, j3))
    assert -math.pi <= r1 <= math.pi
    assert -math.pi <= r2 <= math.pi
    assert -math.pi <= r3 <= math.pi
