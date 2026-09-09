from scservo_sdk import sms_sts, PortHandler, COMM_SUCCESS
# Class attributes (shared by all instances)
BAUD_RATE = 1000000
SERIAL_PORT = '/dev/ttyACM0'

# ConfigurationDDR_ID = 5
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

# Initialize the port handler and servo handler
port_handler = PortHandler(SERIAL_PORT)
servo = sms_sts(port_handler)

if port_handler.openPort():
    print("\nPort opened\n")
else:
    print("✗ Failed to open port")

def find_id():
    for i in range(20):
        model_number, result, error = servo.ping(i)
        if result == COMM_SUCCESS :
            return i

def set_id(id):
    old_id = find_id()
    new_id = id

    if old_id == new_id :
        print("같은 id 로 변경 할 수 없습니다.")
        return

    result, _ = servo.write1ByteTxRx(old_id,ADDR_LOCK,0)
    if result == COMM_SUCCESS:
        print("잠금 해제 성공")
    else :
        return
    
    result, _ = servo.write1ByteTxRx(old_id,SMS_STS_ID, new_id)
    if result == COMM_SUCCESS :
        print(f"id 가 정상적으로 변경 되었습니다. {old_id} -> {new_id}")
    else :
        return

    result, _ = servo.write1ByteTxRx(new_id,ADDR_LOCK,1)
    if result == COMM_SUCCESS :
        print("잠금 성공")
    else :
        return
    

def main():
    set_id(6)


if __name__ == "__main__":
    main()
    port_handler.closePort()
    print("\nPort closed\n")
