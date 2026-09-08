from scservo_sdk import sms_sts, PortHandler, COMM_SUCCESS
import numpy as np
import time
import json
from convert import shift2raw, raw2shift, shift2rad, raw2deg, deg2shift

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

POSITION_MIN = 0
POSITION_MAX = 4095

IDS = [1, 2, 3, 4, 5, 6]


port_handler = PortHandler(SERIAL_PORT)
servo = sms_sts(port_handler)

port_handler.openPort()

def set_servo_torque(status = 0):
    if status == 1:
        for id in IDS:
            data, _, _ = servo.read1ByteTxRx(id, ADDR_TORQUE_ENABLE)

            if  data == 1:
                print(f"{id}'s torque is ready")
            else:
                servo.write1ByteTxRx(id, ADDR_TORQUE_ENABLE,1)
                print(f"{id}'s torque is ready")
    else:
        for id in IDS:
            data, _, _ = servo.read1ByteTxRx(id, ADDR_TORQUE_ENABLE)

            if  data == 0:
                print(f"{id}'s torque is disable")
            else:
                servo.write1ByteTxRx(id, ADDR_TORQUE_ENABLE,0)
                print(f"{id}'s torque is disable")

def servo_check():

    for id in IDS:
        model_number, comm_result, error = servo.ping(id)
        if comm_result == COMM_SUCCESS:
            # print(f"{i+1} is ready : {model_number}")
            pass
        else :
            print(f"{id} failed : {model_number}")
    print (f"\nEvery servo is ready!!\n")

def move_to_degree(id,degree, speed=500, acc=15):
    
    raw, _, _ = servo.ReadPos(id)
    c_shift = raw2shift(id,raw)

    print(raw2deg(id,raw))

    shift = deg2shift(degree)
    if c_shift == shift :
        print("already here")
        return
    
    set_servo_torque(1)
    
    mode, _, _ = servo.read1ByteTxRx(id,ADDR_MODE)
    if mode != 0:
        servo.write1ByteTxRx(id,ADDR_MODE,0)

    with open("calibration_offset.json", "r", encoding='utf-8') as f:
        data = json.load(f)
    s_max,s_min = data[str(id)]["shifted_max"], data[str(id)]["shifted_min"]
    shift = max(s_min,min(s_max,shift))
    
    raw = shift2raw(id,shift)
    servo.WritePosEx(id, raw, speed, acc)
    

def main():
    servo_check()

    id = 1
    goal = 0
    move_to_degree(id,goal,500)
    time.sleep(0.05)

    while True:
        data, _, _ = servo.read1ByteTxRx(id,ADDR_MOVING)
        raw = servo.ReadPos(id)[0]
        
        if data == 0:
            print("goal")
            break
        # print("is moving!")
        print(f"current: {raw2deg(id,raw):.2f}, diff: {np.abs(goal - raw2deg(id,raw)):.2f}")
        time.sleep(0.05)
    
    


if __name__ == "__main__":
    try:
        main()
    finally:
        set_servo_torque(0)
        port_handler.closePort()
        print("\nPort closed\n")
