import numpy as np

# raw ⟷ deg/rad 변환
# 중앙값: 2048 -> 서보 EEPROM 

MAX_RAW = 4096     
CENTER = MAX_RAW // 2


def raw2rad(raw):
    return (raw - CENTER) * 2 * np.pi / MAX_RAW

def rad2raw(rad):
    return int(round(rad * MAX_RAW / (2 * np.pi))) + CENTER

def raw2deg(raw):
    return (raw - CENTER) * 360 / MAX_RAW

def deg2raw(deg):
    return int(round(deg * MAX_RAW / 360)) + CENTER
