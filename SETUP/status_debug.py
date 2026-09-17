from scservo_sdk import sms_sts, PortHandler, COMM_SUCCESS

from SETUP.src.servo import Arm
from SETUP.src.config import *

import time


if __name__ == "__main__":
    with Arm() as arm :
        # while True:
        #     msg = ""
        #     for id in IDS:
        #         msg += f" {id}: {arm.read_pos(id):.2f} |"
        #     print(msg)
        #     time.sleep(0.05)

        arm.servo.write1ByteTxRx(6,ADDR_TORQUE_ENABLE,128)