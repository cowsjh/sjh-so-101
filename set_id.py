import sys
from src.servo import Arm

# 서보 하나만 연결한 상태에서 그 서보의 id를 부여

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("사용법: python set_id.py <새_id>   (서보 하나만 연결한 상태에서)")
        sys.exit(1)
    new_id = int(sys.argv[1])

    arm = Arm()
    arm.port_handler.openPort()
    print("\nPort opened\n")
    try:
        arm.set_id(new_id)
    finally:
        arm.port_handler.closePort()
        print("\nPort closed\n")
