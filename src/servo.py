from scservo_sdk import sms_sts, PortHandler, COMM_SUCCESS
import json, time
from src.config import *
from src.convert import *

class Arm :
    def __init__(self, port = SERIAL_PORT):
        self.port_handler = PortHandler(port)
        self.servo = sms_sts(self.port_handler)
        self.calib = self.load_calib()

    def __enter__(self):
        self.port_handler.openPort()
        self.ping_check()
        self.torque_check()
        return self

    def __exit__(self, *exc):
        for id in IDS:
            self.set_torque(id,0)
        self.port_handler.closePort()
        print("\nPort closed\n")

    def ping_check(self):
        ok = True
        for id in IDS:
            model_number, comm_result, error = self.servo.ping(id)
            ok &= comm_result == COMM_SUCCESS
            if comm_result == COMM_SUCCESS:
                pass
            else :
                print(f"{id} failed : {model_number}")

        if ok :
            print ("ping - ok")

    def torque_check(self):
        ok = True
        for id in IDS:
            data, _, _ = self.servo.read1ByteTxRx(id, ADDR_TORQUE_ENABLE)
            ok &= data == 1

        if ok:
            print("torque - on")

    # init setting

    def get_id(self) -> int:
        for i in range(20):
            model_number, result, error = self.servo.ping(i)
            if result == COMM_SUCCESS :
                return i

    def set_id(self,id):
        old_id = self.get_id()
        new_id = id

        if old_id == new_id :
            print("같은 id 로 변경 할 수 없습니다.")
            return

        result, _ = self.servo.write1ByteTxRx(old_id,ADDR_LOCK,0)
        if result == COMM_SUCCESS:
            pass
        else :
            print("잠금 해제 실패")
            return
        
        result, _ = self.servo.write1ByteTxRx(old_id,SMS_STS_ID, new_id)
        if result == COMM_SUCCESS :
            print(f"id 가 정상적으로 변경 되었습니다. {old_id} -> {new_id}")
        else :
            print(f"id 변경 실패. {old_id} -> {new_id}")
            return

        result, _ = self.servo.write1ByteTxRx(new_id,ADDR_LOCK,1)
        if result == COMM_SUCCESS :
            pass
        else :
            print("잠금 실패")
            return

    def set_torque(self, id, on=0):
        self.servo.write1ByteTxRx(id, ADDR_TORQUE_ENABLE, 1 if on else 0)

    def read_pos(self,id):
        return self.servo.ReadPos(id)[0]

    def load_calib(self):
        with open(CALIB_FILE, encoding="utf-8") as f:
            return json.load(f)

    def save_calib(self):
        with open(CALIB_FILE, "w", encoding="utf-8") as f:
            json.dump(self.calib, f, indent=4)

    # calibration

    def set_raw_limits(self, ids):
        for id in ids:                            # 토크 풀고 리셋
            self.set_torque(id, on=False)
            self.calib[str(id)]["raw_min"] = 9999
            self.calib[str(id)]["raw_max"] = -9999

        try:
            while True:                           # 손으로 관절을 끝까지 돌리면 min/max 갱신
                for id in ids:
                    raw = self.read_pos(id)
                    lo = min(self.calib[str(id)]["raw_min"], raw)
                    hi = max(self.calib[str(id)]["raw_max"], raw)
                    self.calib[str(id)]["raw_min"] = lo
                    self.calib[str(id)]["raw_max"] = hi
                    print(f"id {id}: raw {raw}  ({lo}~{hi})")
                    time.sleep(0.05)
        except KeyboardInterrupt:                 # Ctrl+C 로 수집 종료
            self.save_calib()                     # ← 여기서 한 번만 저장
            for id in ids:
                c = self.calib[str(id)]
                print(f"{id}: {c['raw_min']} ~ {c['raw_max']}")

    def write_current_raw(self, id, key, value=None):
            
            if value is not None:
                self.calib[str(id)][key] = value
            else:
                self.calib[str(id)][key] = self.servo.ReadPos(id)[0]

            self.save_calib()

    def set_center(self, id):

        if self.servo.read1ByteTxRx(id,ADDR_TORQUE_ENABLE)[0] == 1:
            self.servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,0)

        _min = self.calib[str(id)]["raw_min"]
        _max = self.calib[str(id)]["raw_max"]

        mid = int(_min+(((_min-_max)**2)**0.5 * 0.5))

        print(f"{_min} - {_max} = {mid}")
        # raw, _, _ = self.servo.ReadPos(id)
        
        if self.read_pos(id) == mid:
            pass
        else:
            if self.servo.read1ByteTxRx(id,ADDR_TORQUE_ENABLE)[0]==0:
                self.servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,1)

            self.servo.WritePosEx(id,mid,100,15)
            done = 0
            while done == 0:
                current_raw = self.servo.ReadPos(id)[0]
                print(current_raw)
                time.sleep(0.05)
                if abs(current_raw-mid) < 5:

                    diff = current_raw - int((POSITION_MAX+1)/2)

                    self.calib[str(id)]["raw_max"] += diff
                    self.calib[str(id)]["raw_min"] -= diff

                    self.save_calib()

                    self.servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,128)
                    print(f"now {id}'s middle is {self.read_pos(id)}")
                    self.servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,0)

                    return