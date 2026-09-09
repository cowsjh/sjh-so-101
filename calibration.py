from scservo_sdk import sms_sts, PortHandler, COMM_SUCCESS
import numpy as np
import time
import json
from src.convert import  raw2deg, raw2deg
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

def write_current_raw(id, key, value=None, json_file="calibration_offset.json"):
    with open(json_file, "r", encoding='utf-8') as f:
        data = json.load(f)
    
    if value is not None:
        data[str(id)][key] = value
    else:
        data[str(id)][key] = servo.ReadPos(id)[0]

    with open(json_file, "w", encoding='utf-8') as f:
        json.dump(data, f, indent=4)


def set_raw_limits(ids,json_file= "calibration_offset.json"):

    with open(json_file, "r", encoding='utf-8') as f:
        data = json.load(f)

    
    #reset
    for id in ids:
        
        if servo.read1ByteTxRx(id,ADDR_TORQUE_ENABLE)[0]:
            servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,0)

        data[str(id)]["raw_min"] = 9999
        data[str(id)]["raw_max"] = -9999

    try:
        while True:
            for id in ids:
                # id = 1
                raw, _, _ = servo.ReadPos(id)
                print(f"id: {id}, raw: {raw}")
                data[str(id)]["raw_min"] = min(data[str(id)]["raw_min"],raw)
                data[str(id)]["raw_max"] = max(data[str(id)]["raw_max"],raw)
                # pose[id] = min_shift,max_shift
                time.sleep(0.05)
    except KeyboardInterrupt:
        with open(json_file, "w", encoding='utf-8') as f:
            json.dump(data, f, indent=4)
        
        for id in ids:
            print(f"\n{id} : raw: {data[str(id)]['raw_min']}, {data[str(id)]['raw_max']}")

        
def set_center (id,json_file= "calibration_offset.json"):

    if servo.read1ByteTxRx(id,ADDR_TORQUE_ENABLE)[0] == 1:
        servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,0)

    with open(json_file, "r", encoding='utf-8') as f:
        data = json.load(f)
    
    min = data[str(id)]["raw_min"]
    max = data[str(id)]["raw_max"]

    mid = int(min+(((min-max)**2)**0.5 * 0.5))

    print(f"{min} - {max} = {mid}")
    # raw, _, _ = servo.ReadPos(id)
    
    if servo.ReadPos(id) == mid:
        pass
    else:
        if servo.read1ByteTxRx(id,ADDR_TORQUE_ENABLE)[0]==0:
            servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,1)
        servo.WritePosEx(id,mid,100,15)
        done = 0
        while done == 0:
            current_raw = servo.ReadPos(id)[0]
            print(current_raw)
            time.sleep(0.05)
            if abs(current_raw-mid) < 5:

                diff = current_raw - int((RATIO_MAX+1)/2)

                data[str(id)]["raw_max"] += diff
                data[str(id)]["raw_min"] -= diff

                with open(json_file, "w", encoding='utf-8') as f:
                    json.dump(data, f, indent=4)

                servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,128)
                print(f"now {id}'s middle is {servo.ReadPos(id)[0]}")
                servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,0)

                return
        # if servo.ReadPos(id) == mid:

    # raw, _, _ = servo.ReadPos(id)
    # print (f"changed: {raw}")
    


def main():
    servo_counts = 6
    servo_check(servo_counts)
    # set_center(6)
    # servo.write1ByteTxRx(6,ADDR_TORQUE_ENABLE,128)
    # set_raw_limits([6])
    # set_raw_limits(IDS)

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


    # id = 6
    # try:
    #     while True:
    #         raw, _, _ = servo.ReadPos(id)
        
    #         print(f"raw: {raw}, degrees: {raw2deg(raw):.2f}")
    #         time.sleep(0.05)
    # except KeyboardInterrupt:
    #     return


    # raw, _, _ = servo.ReadPos(id)
    # print (f"current: {raw}")

    # set_raw_limits(IDS)

    # raw, _, _ = servo.ReadPos(id)
    # print (f"changed: {raw}")

    # write_current_raw(id,"offset",2048-raw)
    # set_shifted_limits([6])

    # while True:
    #     raw, _, _ = servo.ReadPos(id)
    #     print (f"current: {raw2deg(id,raw)}")
    #     time.sleep(0.05)

    

    



if __name__ == "__main__":
    try:
        main()
    finally:
        ok = True
        for id in IDS:
            if servo.read1ByteTxRx(id,ADDR_TORQUE_ENABLE)[0] == 1:
                servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,0)   
            ok &= servo.read1ByteTxRx(id,ADDR_TORQUE_ENABLE)[0] == 0

        if ok:
            print("\nevery torque is disabled")
        port_handler.closePort()
        print("\nPort closed\n")
