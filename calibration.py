from scservo_sdk import sms_sts, PortHandler, COMM_SUCCESS
import numpy as np
import time
import json
from convert import raw2shift, shift2raw, shift2rad, deg2shift, raw2deg
## ID 6 아래2655~위4079
## ID 5 

BAUD_RATE = 1000000
SERIAL_PORT = "/dev/ws-servo-board"
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

SMS_STS_ID = 5

RATIO_MIN = 0
RATIO_MAX = 4095

IDS = [1, 2, 3, 4, 5, 6]


port_handler = PortHandler(SERIAL_PORT)
servo = sms_sts(port_handler)

port_handler.openPort()

def servo_check(servo_counts):

    for i in range(servo_counts):
        model_number, comm_result, error = servo.ping(i+1)
        if comm_result == COMM_SUCCESS:
            # print(f"{i+1} is ready : {model_number}")
            pass
        else :
            print(f"{i+1} failed : {model_number}")
    print (f"\nEvery servo is ready!!\n")

# 변환 함수(raw/shift/deg)는 convert.py로 분리됨 → 상단에서 import

def write_current_raw(id, key, value=None, json_file="calibration.json"):
    with open(json_file, "r", encoding='utf-8') as f:
        data = json.load(f)
    
    if value is not None:
        data[str(id)][key] = value
    else:
        data[str(id)][key] = servo.ReadPos(id)[0]

    with open(json_file, "w", encoding='utf-8') as f:
        json.dump(data, f, indent=4)


def set_shifted_limits(ids):
    with open("calibration_offset.json", "r", encoding='utf-8') as f:
        data = json.load(f)

    shifted = {}
    for id in IDS:
        shifted[id] = 9999, -9999
    try:
        while True:
            for id in ids:
                # id = 1
                pos, _, _ = servo.ReadPos(id)
                shift = raw2shift(id, pos)
                print(f"id: {id}, raw: {pos}, shift: {shift}, deg: {np.rad2deg(shift2rad(id, shift))}")
                min_shift = min(shifted[id][0],shift)
                max_shift = max(shifted[id][1],shift)
                shifted[id] = min_shift,max_shift
                time.sleep(0.05)
    except KeyboardInterrupt:
        # print(shifted)
        for id in ids:
            # id = 1
            write_current_raw(id, "shifted_min", shifted[id][0], "calibration_offset.json")
            write_current_raw(id, "shifted_max", shifted[id][1], "calibration_offset.json")
            
            print(f"\n{id} : shift: {shifted[id][0]}, {shifted[id][1]}, deg: {np.rad2deg(shift2rad(id, shifted[id][0]))}, {np.rad2deg(shift2rad(id, shifted[id][1]))}")
        
def set_EEPROM (id, ADDR, data, result=False):
    servo.write1byte(id,ADDR,data)
    
    
def main():
    servo_counts = 6
    servo_check(servo_counts)

    # torque disable
    # for i in range(6):
    #     data, _, _ = servo.read1ByteTxRx(i+1, ADDR_TORQUE_ENABLE)
    #     if  data == 1:
    #         servo.write1ByteTxRx(i+1, ADDR_TORQUE_ENABLE,0)
    #         print(f"{i+1} torque disabled")
    #     else :
    #         print(f"{i+1}'s torque is already disabled")

    # set_shifted_limits([2])
    # while True:
    #     raw, _, _ = servo.ReadPos(1)
    #     shift = raw2shift(1,raw)
    #     rad = shift2rad(1, shift)
    #     print(f"{raw}, {shift}, {rad:.2f}, {np.rad2deg(rad):.2f}")
    #     time.sleep(0.05)


    id = 2
    try:
        while True:
            raw, _, _ = servo.ReadPos(id)
            shift = raw2shift(id,raw)
            print(f"raw: {raw}, shift: {shift}, rad: {shift2rad(1,shift):.2f}, degrees: {raw2deg(id,raw):.2f}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    try:
        main()
    finally:
        port_handler.closePort()
        print("\nPort closed\n")
