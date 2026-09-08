import json
import numpy as np

# raw ⟷ shift ⟷ deg 변환 계층 (하드웨어/포트 없음, 순수 함수).
# offset(서보 개별성)은 raw ⟷ shift 경계에서만 붙는다 → 그 함수만 id를 받음.
# calibration.py / sync_write.py 양쪽에서 import.


def raw2shift(id, raw, max_raw=4096):
    with open("calibration_offset.json", "r", encoding='utf-8') as f:
        data = json.load(f)
    offset = data[str(id)]["offset"]

    shift = (raw + offset) % max_raw
    return shift

def shift2raw(id, shift, max_raw=4096):
    with open("calibration_offset.json", "r", encoding='utf-8') as f:
        data = json.load(f)
    offset = data[str(id)]["offset"]

    return (shift - offset) % max_raw


def shift2rad(shift, max_raw=4096):
    return ((shift - max_raw // 2) * 2 * np.pi) / max_raw


def deg2shift(deg, max_raw=4096):
    return int(deg * max_raw / 360) + (max_raw // 2)


def raw2deg(id, raw, max_raw=4096):
    return np.rad2deg(shift2rad(raw2shift(id, raw, max_raw), max_raw))

def def2raw(id,deg,max_raw=4096):
    return shift2raw(id,deg2shift(deg,max_raw),max_raw)