import sys, time
from collections import deque
from src.servo import Arm
from src.config import (
    IDS, POSITION_MIN, POSITION_MAX, ADDR_MODE, CALIB_FILE, CALIB_FILE_LEADER,
    ADDR_PRESENT_LOAD, ADDR_PRESENT_TEMPERATURE,
)

# 자동 캘리브레이션: 각 관절을 모터로 +/- 하드스톱까지 밀고, 스톨(위치 안 나감)로 끝 감지.
# ID 1~5 (그리퍼 6 제외). 양방향 raw_min/max 잡고 → 기존 set_center 로 중앙 재각인.

AUTO_IDS = [1, 2, 3, 4, 5]

# ── 튜닝 노브 (하드웨어마다 조정) ──────────────────────────────
STEP         = 150   # 램프 한 스텝당 target 전진량(raw). 클수록 빨리 민다
SPEED        = 300   # WritePosEx 속도 (느리게=부드럽게)
ACC          = 30
ERR_MAX      = 400   # target이 cur을 앞서는 최대치(추종오차 상한). 막히면 이만큼 오차=토크로 breakaway.
                     #  STEP만큼만 앞서던 옛 방식은 뻑뻑/중력 관절을 못 넘겨 오검했음. 하드스톱 쾅 박기도 방지.
WINDOW       = 5     # 최근 WINDOW 샘플의 순이동으로 이동 여부 판정(순간값 아님)
WINDOW_MIN   = 12    # 창 전체 순이동 raw. 이 밑 = 안 움직임
STALL_LOAD   = 500   # 스톱 확정 load 문턱(<OVERLOAD). '안 움직임 AND load 높음'이라야 하드스톱.
                     #  breakaway/중력 지연은 load 아직 낮음 → 통과. 하드스톱 정상부하가 이 값 넘게 ERR_MAX 튜닝.
STALL_LIM    = 3     # '안 움직임 & load 높음' 연속 이만큼(≈0.3s) → 스톱 확정
STUCK_HARD   = 40    # 안전망: load 문턱 미달이어도 안 움직임 이만큼 연속(≈4s)이면 정지+경고(STALL_LOAD 재튜닝)
MARGIN       = 15    # 기록 시 스톱보다 살짝 안쪽(소프트리밋이 기계스톱 안 침)
DELAY        = 0.1   # 스텝 간 대기 (초)
# 비상정지(안전) — 스톱 판정과 별개
TEMP_MAX     = 65    # ℃, 과열 비상정지
OVERLOAD     = 980   # 최대토크 근처(하위10비트)
OVERLOAD_LIM = 15    # OVERLOAD 가 이만큼 연속(=1.5s) 지속되면 진짜 잼 → 비상정지
CENTER_TOL   = 10    # mid 도달 허용오차 raw
CENTER_TMO   = 4.0   # mid 이동 타임아웃 (초, 무한루프 방지)
OFFCENTER    = 80    # |mid-2048| 이 이상이면 조립 중심 어긋남 경고
# ─────────────────────────────────────────────────────────────


def _drive_to_stop(arm: Arm, id: int, direction: int) -> int:
    """direction=+1/-1. 하드스톱까지 밀고 멈춘 raw 반환.
    핵심 두 가지:
      ① target을 cur 추종이 아니라 단조 램프시키되 cur+ERR_MAX 이상 앞서지 않게 캡한다.
         → 막히면 추종오차가 ERR_MAX까지 쌓여 토크가 커지고 마찰/중력 breakaway를 넘긴다.
         (옛 방식은 target을 cur+STEP에 고정 → 토크 상한 STEP에 갇혀 못 넘기고 그 자리서 오검)
      ② 스톱 판정 = '창 순이동 미미 AND load 높음'이 STALL_LIM 연속.
         움직이는 중엔 net>WINDOW_MIN이라 걸러지고(팔 무게 load 스파이크 무관),
         안 움직이는데 load 아직 낮으면 breakaway 지연으로 보고 계속 민다.
      load 는 이 판정과 온도·지속최대토크 비상정지에만 쓴다."""
    over = 0
    stuck = 0        # 안 움직임 & load 높음 연속 카운트
    low_move = 0     # 안 움직임(load 무관) 연속 카운트 — 안전망용
    hist = deque(maxlen=WINDOW + 1)
    cur = arm.read_pos(id)
    target = cur
    hist.append(cur)
    while True:
        target += direction * STEP                       # 단조 램프
        lead_cap = cur + direction * ERR_MAX             # cur 앞서는 양 상한
        target = min(target, lead_cap) if direction > 0 else max(target, lead_cap)
        target = max(POSITION_MIN, min(POSITION_MAX, target))
        arm.servo.WritePosEx(id, target, SPEED, ACC)
        time.sleep(DELAY)
        prev = cur
        cur = arm.read_pos(id)

        load, _, _ = arm.servo.read2ByteTxRx(id, ADDR_PRESENT_LOAD)
        temp, _, _ = arm.servo.read2ByteTxRx(id, ADDR_PRESENT_TEMPERATURE)
        mag = load & 0x3FF                      # 하위10비트=크기
        step_prog = (cur - prev) * direction
        print(f"  id {id} dir{direction:+d} target={target} cur={cur} prog={step_prog} load={mag} temp={temp}")

        # 비상정지(안전) — 스톱 판정과 별개
        if temp > TEMP_MAX:
            print(f"  ⚠ id {id} 과열 temp={temp} → 비상정지"); return cur
        over = over + 1 if mag >= OVERLOAD else 0
        if over >= OVERLOAD_LIM:
            print(f"  ⚠ id {id} 최대토크 {over}회 지속(잼 의심) → 비상정지"); return cur

        # 스톱 판정 = 창 순이동 미미 AND load 높음, STALL_LIM 연속
        hist.append(cur)
        if len(hist) == hist.maxlen:
            net = (hist[-1] - hist[0]) * direction
            if net < WINDOW_MIN:
                low_move += 1
                stuck = stuck + 1 if mag >= STALL_LOAD else 0
                if stuck >= STALL_LIM:
                    print(f"  id {id} 순이동 {net}<{WINDOW_MIN} & load {mag}≥{STALL_LOAD} {stuck}연속 → 스톱 확정")
                    return cur
                if low_move >= STUCK_HARD:   # 안전망: load 문턱 미달인데 계속 안 움직임 → 강제정지
                    print(f"  ⚠ id {id} load<{STALL_LOAD}인데 {low_move}회 정지 상태 → 강제정지(STALL_LOAD 재튜닝 검토)")
                    return cur
            else:
                stuck = 0; low_move = 0

        if target in (POSITION_MIN, POSITION_MAX):   # 스톱 없이 raw 한계 도달(연속관절 방어)
            print(f"  id {id} raw 한계 {target} 도달 → 정지")
            return cur


def _go_center(arm: Arm, id: int, raw_min: int, raw_max: int):
    """진짜 스톱 min/max의 중점으로 이동. min/max 는 실제 raw 그대로 둠(오프셋 뒤섞기 X)."""
    mid = (raw_min + raw_max) // 2
    arm.set_torque(id, 1)
    arm.servo.WritePosEx(id, mid, SPEED, ACC)
    t0 = time.time()
    while time.time() - t0 < CENTER_TMO:
        if abs(arm.read_pos(id) - mid) < CENTER_TOL:
            break
        time.sleep(DELAY)
    cur = arm.read_pos(id)
    off = cur - 2048
    warn = "  ⚠ 조립 중심 어긋남(혼 재장착 검토)" if abs(off) > OFFCENTER else ""
    print(f" id {id} center → raw={cur} mid={mid} (2048 대비 {off:+d}){warn}")


def auto_calibrate(arm: Arm, id: int):
    print(f"\n=== id {id} 자동 캘리브 ===")
    arm.set_torque(id, 1)
    if arm.servo.read1ByteTxRx(id, ADDR_MODE)[0] != 0:
        arm.servo.write1ByteTxRx(id, ADDR_MODE, 0)   # 위치모드 보장

    print(" + 방향 스윕")
    max_stop = _drive_to_stop(arm, id, +1)
    print(" - 방향 스윕")
    min_stop = _drive_to_stop(arm, id, -1)

    raw_max = max_stop - MARGIN
    raw_min = min_stop + MARGIN
    arm.calib[str(id)]["raw_min"] = raw_min
    arm.calib[str(id)]["raw_max"] = raw_max
    arm.save_json(arm.calib_file)
    print(f" id {id}: raw_min={raw_min} raw_max={raw_max} (스톱 {min_stop}~{max_stop}, margin {MARGIN})")

    _go_center(arm, id, raw_min, raw_max)   # 자체 중앙이동(set_center 안 씀 — 오프셋 뒤섞기·무한루프 회피)
    arm.set_torque(id, 1)       # 안전: 캘리 끝난 조인트는 토크 유지(자세 홀드, sag 방지)
    print(f" id {id} 완료 — 토크 유지 중")


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else None
    ans = input(f"\n⚠ ID {AUTO_IDS} 관절이 스스로 하드스톱까지 움직입니다. 주변 치우고 진행? [y/N] ").strip().lower()
    if ans != "y":
        print("취소"); sys.exit(0)

    calib_file = CALIB_FILE_LEADER if (port and "leader" in port) else CALIB_FILE
    print(f"calib 파일: {calib_file}")
    arm_ctx = Arm(port, calib_file) if port else Arm(calib_file=calib_file)
    with arm_ctx as arm:
        for id in AUTO_IDS:
            auto_calibrate(arm, id)

        print("\n=== 전체 캘리 완료 ===")
        for id in AUTO_IDS:
            c = arm.calib[str(id)]
            print(f" id {id}: raw_min={c['raw_min']} raw_max={c['raw_max']}")
        print("그리퍼(6)는 자동 제외 — 필요시 손으로.")
        print("캘리된 조인트 토크 유지 중. q + Enter 또는 Ctrl-C 로 종료.")
        try:
            while input().strip().lower() != "q":
                pass
        except KeyboardInterrupt:
            pass
    # with 종료 → __exit__ 이 전 축 토크 off (여기서 팔이 풀림)
