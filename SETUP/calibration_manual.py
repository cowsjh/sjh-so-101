import sys, time, threading
from src.servo import Arm
from src.config import (
    IDS, POSITION_MIN, POSITION_MAX, CALIB_FILE, CALIB_FILE_LEADER,
)
from src.convert import CENTER

# 수동 캘리브레이션:
#  1) id 1~5 토크 OFF → 손으로 각 관절 min/max 끝까지 왕복 (실시간 피드백 표시)
#  2) Enter → 잡은 min/max 를 json 에 쓰고, 중앙값으로 id 1→5 이동 후 토크 고정
#  3) Enter → 종료 (with 종료 시 전 축 토크 off, 팔 풀림)

CAL_IDS = [1, 2, 3, 4, 5]   # 그리퍼(6) 제외

SPEED    = 300
ACC      = 30
POLL     = 0.05   # sweep 샘플 주기 (초)
CTR_TMO  = 4.0    # 중앙 이동 타임아웃 (초/관절)
CTR_TOL  = 15     # 중앙 도달 허용오차 raw
OFFCENTER = 80    # |mid-CENTER| 이 이상이면 조립 중심 어긋남 경고


def _read_valid(arm: Arm, id: int):
    """ReadPos 는 무응답 서보에 음수/쓰레기를 낼 수 있음. 유효범위 밖이면 None."""
    p = arm.read_pos(id)
    return p if POSITION_MIN <= p <= POSITION_MAX else None


def sweep(arm: Arm):
    """손 왕복 동안 관절별 min/max 갱신. Enter 누르면 종료."""
    for id in CAL_IDS:
        arm.set_torque(id, 0)   # 손으로 움직이게 토크 해제

    lo = {id: None for id in CAL_IDS}
    hi = {id: None for id in CAL_IDS}

    stop = threading.Event()
    threading.Thread(
        target=lambda: (input("\n>> id 1~5 를 각각 끝까지 손으로 왕복시키세요. 다 되면 Enter.\n"), stop.set()),
        daemon=True,
    ).start()

    while not stop.is_set():
        cells = []
        for id in CAL_IDS:
            p = _read_valid(arm, id)
            if p is None:
                cells.append(f"{id}: ---무응답---")
                continue
            lo[id] = p if lo[id] is None else min(lo[id], p)
            hi[id] = p if hi[id] is None else max(hi[id], p)
            cells.append(f"{id}:{p:4d}[{lo[id]:4d}~{hi[id]:4d}]")
        print("  ".join(cells), end="\r", flush=True)
        time.sleep(POLL)
    print()

    missing = [id for id in CAL_IDS if lo[id] is None]
    if missing:
        print(f"⚠ id {missing} 응답 없음 — 배선/ID 확인 필요. 이 관절은 건너뜀.")
    return lo, hi


def go_center(arm: Arm, id: int, mid: int):
    arm.set_torque(id, 1)
    arm.servo.WritePosEx(id, mid, SPEED, ACC)
    t0 = time.time()
    while time.time() - t0 < CTR_TMO:
        p = _read_valid(arm, id)
        if p is not None and abs(p - mid) < CTR_TOL:
            break
        time.sleep(POLL)
    cur = _read_valid(arm, id)
    off = (cur - CENTER) if cur is not None else None
    warn = "  ⚠ 조립 중심 어긋남(혼 재장착 검토)" if off is not None and abs(off) > OFFCENTER else ""
    shown = f"raw={cur} (CENTER 대비 {off:+d})" if cur is not None else "raw=무응답"
    print(f" id {id} center → mid={mid} {shown}{warn}")


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else None
    calib_file = CALIB_FILE_LEADER if (port and "leader" in port) else CALIB_FILE
    print(f"calib 파일: {calib_file}")
    arm_ctx = Arm(port, calib_file) if port else Arm(calib_file=calib_file)

    with arm_ctx as arm:
        lo, hi = sweep(arm)

        # min/max json 기록
        for id in CAL_IDS:
            if lo[id] is None:
                continue
            arm.calib[str(id)]["raw_min"] = lo[id]
            arm.calib[str(id)]["raw_max"] = hi[id]
        arm.save_json(arm.calib_file)
        print(f"\n{arm.calib_file} 저장 완료")

        # 중앙값으로 id 1→5 이동 + 토크 고정
        print("\n=== 중앙 이동 ===")
        for id in CAL_IDS:
            if lo[id] is None:
                print(f" id {id} 건너뜀(무응답)"); continue
            mid = (lo[id] + hi[id]) // 2
            print(f" id {id}: raw_min={lo[id]} raw_max={hi[id]} mid={mid}")
            go_center(arm, id, mid)

        print("\n중앙 정렬 + 토크 고정 완료. Enter 로 종료(토크 풀림).")
        try:
            input()
        except KeyboardInterrupt:
            pass
    # with 종료 → __exit__ 이 전 축 토크 off
