import sys
from scservo_sdk import sms_sts, PortHandler, COMM_SUCCESS
import numpy as np
import time
import json
from src.convert import rad2raw, raw2deg, deg2raw, raw2rad

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

def set_servo_torque(id, status = 0):
    if status == 1:
        data, _, _ = servo.read1ByteTxRx(id, ADDR_TORQUE_ENABLE)

        if  data == 1:
            print(f"{id}'s torque is ready")
        else:
            servo.write1ByteTxRx(id, ADDR_TORQUE_ENABLE,1)
            print(f"{id}'s torque is ready")
    else:
        data, _, _ = servo.read1ByteTxRx(id, ADDR_TORQUE_ENABLE)

        if  data == 0:
            print(f"{id}'s torque is disable")
        else:
            servo.write1ByteTxRx(id, ADDR_TORQUE_ENABLE,0)
            print(f"{id}'s torque is disable")

def torque_check(ids):
    msg = "torque: "
    for id in ids:
        data, _, _ = servo.read1ByteTxRx(id, ADDR_TORQUE_ENABLE)
        msg += f"| {id} = {data==1} |"
    print(msg)

def servo_check():

    for id in IDS:
        model_number, comm_result, error = servo.ping(id)
        if comm_result == COMM_SUCCESS:
            # print(f"{i+1} is ready : {model_number}")
            pass
        else :
            print(f"{id} failed : {model_number}")
    print (f"\nEvery servo is ready\n")

def move_to_degree(id,degree, speed=500, acc=15):
    
    c_raw, _, _ = servo.ReadPos(id)
    c_deg = raw2deg(c_raw)


    t_raw = deg2raw(degree)
    t_deg = degree
    

    if abs(t_deg - c_deg) < 1  :
        print("already here")
        return
    
    print(f"raw: {c_raw}, deg: {c_deg:.2f}")
    set_servo_torque(id,1)
    
    mode, _, _ = servo.read1ByteTxRx(id,ADDR_MODE)
    if mode != 0:
        servo.write1ByteTxRx(id,ADDR_MODE,0)

    with open("calibration_offset.json", "r", encoding='utf-8') as f:
        data = json.load(f)
    r_max,t_min = data[str(id)]["raw_max"], data[str(id)]["raw_min"]
    t_raw = max(t_min,min(r_max,t_raw))
    
    servo.WritePosEx(id, t_raw, speed, acc)

def move_to_sync(degree_dict, speed = 300, acc = 20):

    for id, degree in degree_dict.items():

        id = int(id)
        degree = float(degree)

        set_servo_torque(id,1)
        raw = deg2raw(degree)

        #clamp min,max
        with open("calibration_offset.json", "r", encoding='utf-8') as f:
            data = json.load(f)
        r_max,r_min = data[str(id)]["raw_max"], data[str(id)]["raw_min"]
        raw = max(r_min,min(r_max,raw))

        servo.SyncWritePosEx(id, raw, speed, acc)

    servo.groupSyncWrite.txPacket()
    servo.groupSyncWrite.clearParam()
    arrived_id = set()
    while True:
        goal = True

        for id, degree in degree_dict.items():
            
            id = int(id)
            degree = float(degree)
            
            t_deg = float(degree)
            c_deg = raw2deg(servo.ReadPos(id)[0])
            is_arrived = abs(t_deg-c_deg) < 2
                
            load, _, _ = servo.read2ByteTxRx(id, ADDR_PRESENT_LOAD)
            curr, _, _ = servo.read2ByteTxRx(id, ADDR_PRESENT_CURRENT)
            temp, _, _ = servo.read2ByteTxRx(id, ADDR_PRESENT_TEMPERATURE)

            print(f"{id:<3} load: {load:<5} curr: {curr:<5} temp: {temp}")

            if is_arrived and id not in arrived_id:

                # print(f"{id} arrived, raw_diff: {abs(t_deg-c_deg)}")
                arrived_id.add(id)
                # print(arrived_id)

            goal &= is_arrived
        print('\n')

        if goal:    
            break

        time.sleep(0.1)

def get_current_pos():
    degree_dict = {}
    for id in IDS:
        raw, _, _ = servo.ReadPos(id)
        degree_dict[str(id)] = raw2deg(raw)
    return degree_dict

def save_current_pose(pose_name):
    with open("poses.json", "r", encoding='utf-8') as f:
        data = json.load(f)

    data[pose_name] = get_current_pos()

    with open("poses.json", "w", encoding='utf-8') as f:
        json.dump(data, f, indent=4)

def main():
    servo_check()

    # id = 6
    # goal = 0
    # move_to_degree(id,goal,300)
    # time.sleep(0.05)

    # while True:
    #     data, _, _ = servo.read1ByteTxRx(id,ADDR_MOVING)
    #     raw = servo.ReadPos(id)[0]
        
    #     if data == 0:
    #         break
    #     # print("is moving!")
    #     print(f"current: {raw2deg(raw):.2f}, diff: {np.abs(goal - raw2deg(raw)):.2f}")
    #     time.sleep(0.02)
    with open("poses.json", "r", encoding='utf-8') as f:
            data = json.load(f)

    move_to_sync(data[sys.argv[1]],speed = 800)
    
    # save_current_pose("home")
    
    



if __name__ == "__main__":
    try:
        main()
    finally:
        
        # for id in IDS:
        #     if servo.read1ByteTxRx(id,ADDR_TORQUE_ENABLE)[0] == 1:
        #         servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,0)   
        
        torque_check(IDS)

        port_handler.closePort()
        print("Port closed\n")
