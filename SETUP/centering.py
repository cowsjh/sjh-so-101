from src.servo import Arm
from src.config import IDS, POSITION_MAX, ADDR_TORQUE_ENABLE
from src.convert import raw2deg, CENTER
from scservo_sdk import COMM_SUCCESS
import sys

# 손으로 각 관절을 센터로 잡아놓고 실행 → 현재 raw를 그 관절의 중앙(mid)으로 찍어줌.
# 이상적 중앙은 CENTER(2048). offset = 지금 raw가 2048에서 얼마나 벗어났나.

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("사용법: python centering.py <port> [id]   (id 생략 시 전체)")
        sys.exit(1)
    port = sys.argv[1]
    ids = [int(sys.argv[2])] if len(sys.argv) > 2 else IDS   # 개별 id 지정 가능

    arm = Arm(port)
    arm.port_handler.openPort()
    print("\nPort opened\n")
    try:
        for id in ids:
            _, result, _ = arm.servo.ping(id)
            if result != COMM_SUCCESS:
                print(f"id {id}: 응답 없음 (skip)")
                continue
            arm.servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,128)

            raw = arm.read_pos(id)
            print(f"id {id}: mid(raw) = {raw:<5} deg = {raw2deg(raw):+7.2f}  offset(from {CENTER}) = {raw - CENTER:+d}")
    finally:
        arm.port_handler.closePort()
        print("\nPort closed\n")
