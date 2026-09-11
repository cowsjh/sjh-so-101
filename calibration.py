import sys, time
from src.servo import Arm
from src.config import IDS


def set_min_max(arm: Arm):
    """손으로 6축을 물리적 끝까지 왕복시키며 raw_min/max 수집. Ctrl+C 로 저장·종료."""
    print("\n>> set_min_max: 6축을 각각 끝까지 천천히 왕복시키세요. 다 되면 Ctrl+C.")
    arm.set_raw_limits([6])          # KeyboardInterrupt 를 내부에서 잡아 저장 후 반환


def set_center(arm: Arm):
    
    ans = input("\n>> set_center: warning! 관절이 중앙값으로 이동 합니다. [y/N] ").strip().lower()
    if ans != "y":
        print("취소")
        return
    # for id in IDS:
        print(f"\n-- id {id} 중앙 재각인 --")
        arm.set_center(id)
        arm.set_torque(id,0)


if __name__ == "__main__":
    # 인자 없으면 전체(min_max -> center), 아니면 단계 지정
    stage = sys.argv[1] if len(sys.argv) > 1 else "all"

    with Arm() as arm:               # __enter__: 포트 open + ping + 토크 상태 / __exit__: 토크 off
        if stage in ("min_max", "all"):
            set_min_max(arm)
        if stage in ("center", "all"):
            set_center(arm)
