import os

BAUD_RATE = 1000000

# SETUP/ 기준 절대경로 — 실행 위치(cwd)와 무관하게 json을 찾음
_SETUP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CALIB_FILE = os.path.join(_SETUP_DIR, "calibration_offset.json")
CALIB_FILE_LEADER = os.path.join(_SETUP_DIR, "calibration_offset_leader.json")
POSES_FILE = os.path.join(_SETUP_DIR, "poses.json")

SERIAL_PORT = "/dev/ws-servo-board"
SERIAL_PORT_LEADER = "/dev/ws-servo-board-leader"
ADDR_MIN_ANGLE_LIMIT = 9
ADDR_MAX_ANGLE_LIMIT = 11
ADDR_MAX_TEMP_LIMIT = 13
ADDR_MAX_VOLTAGE_LIMIT = 14
ADDR_MIN_VOLTAGE_LIMIT = 15
ADDR_LOCK = 55
ADDR_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_SERVO_STATUS = 65
ADDR_MOVING = 66
ADDR_PRESENT_CURRENT = 69

POSITION_MIN = 0
POSITION_MAX = 4095

SMS_STS_ID=5

IDS = [1, 2, 3, 4, 5, 6]
