import sys, time
from src.servo import Arm
from src.config import (
    IDS, POSITION_MIN, POSITION_MAX,
    SERIAL_PORT, SERIAL_PORT_LEADER, CALIB_FILE, CALIB_FILE_LEADER,
)

# 텔레옵 미러 (direct raw copy):
#  두 팔 설계·조립 동일 가정 → leader raw 를 그대로 follower 에 명령.
#  leader 토크 off(손으로 조작) → follower 가 실시간 복제.
#  정규화 안 씀. 서보마다 offset/부호 다르면 SIGN 노브로 축별 보정.

MIRROR_IDS = IDS            # [1..6], 그리퍼 포함
SPEED = 1500                # follower WritePosEx 속도 (raw/s 계열, 반응성↔부드러움)
ACC   = 50
RATE  = 0.02                # 루프 주기 (초) ≈ 50Hz

# 장착방향 보정: 같은 관절회전에 leader/follower raw 가 반대로 움직이면 -1.
# 동일 가정이라 전부 +1. 미러 돌려보고 거꾸로 가는 축만 -1 로.
SIGN = {1: +1, 2: +1, 3: +1, 4: +1, 5: +1, 6: +1}


def _read_valid(arm: Arm, id: int):
    """무응답 서보는 ReadPos 가 범위 밖 값을 냄 → None 으로 걸러 이전 target 유지."""
    p = arm.read_pos(id)
    return p if POSITION_MIN <= p <= POSITION_MAX else None


if __name__ == "__main__":
    l_port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT_LEADER
    f_port = sys.argv[2] if len(sys.argv) > 2 else SERIAL_PORT
    print(f"leader={l_port}  follower={f_port}")

    # 두 버스 동시 오픈. __enter__: open+ping+torque_check / __exit__: 전축 토크off+close
    with Arm(l_port, CALIB_FILE_LEADER) as leader, Arm(f_port, CALIB_FILE) as follower:
        for id in MIRROR_IDS:
            leader.set_torque(id, 0)     # 손으로 움직이게 해제
            follower.set_torque(id, 1)   # 따라오게 고정

        # follower 안전 가동범위 (calib min/max 안쪽으로만 명령)
        lo = {id: follower.calib[str(id)]["raw_min"] for id in MIRROR_IDS}
        hi = {id: follower.calib[str(id)]["raw_max"] for id in MIRROR_IDS}
        for id in MIRROR_IDS:
            if lo[id] > hi[id]:
                lo[id], hi[id] = hi[id], lo[id]

        print("\n>> 미러 시작. leader 손으로 움직이세요. Ctrl-C 로 종료.\n")
        try:
            while True:
                cells = []
                for id in MIRROR_IDS:
                    src = _read_valid(leader, id)
                    if src is None:
                        cells.append(f"{id}:--")
                        continue
                    # 부호 보정 (SIGN=-1 이면 follower 중앙 기준 반전)
                    if SIGN[id] < 0:
                        src = lo[id] + hi[id] - src
                    tgt = max(lo[id], min(hi[id], src))   # 안전 클램프
                    follower.servo.WritePosEx(id, tgt, SPEED, ACC)
                    cells.append(f"{id}:{tgt:4d}")
                print("  ".join(cells), end="\r", flush=True)
                time.sleep(RATE)
        except KeyboardInterrupt:
            print("\n종료")
    # with 종료 → 두 팔 전축 토크 off (follower 풀림)
