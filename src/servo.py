from scservo_sdk import sms_sts, PortHandler, COMM_SUCCESS
import json, time
from src.config import *
from src.convert import *

class Arm :
    def __init__(self, port = SERIAL_PORT):
        self.port_handler = PortHandler(port)
        self.servo = sms_sts(self.port_handler)
        self.calib = self.load_json(CALIB_FILE)
        self.poses = self.load_json(POSES_FILE)

    def __enter__(self):
        self.port_handler.openPort()
        print("\nPort opened\n")

        self.ping_check()
        self.torque_check()
        return self

    def __exit__(self, *exc):
        for id in IDS:
            self.set_torque(id,0)
        self.torque_check()
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

        print("torque state")
        for id in IDS:
            data, _, _ = self.servo.read1ByteTxRx(id, ADDR_TORQUE_ENABLE)
            print(f"{id}: {'on' if data else 'off'}")


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

    def load_json(self, JSON):
        with open(JSON, encoding="utf-8") as f:
            return json.load(f)

    def save_json(self, JSON):
        if JSON == CALIB_FILE :
            with open(JSON, "w", encoding="utf-8") as f:
                json.dump(self.calib, f, indent=4)
        if JSON == POSES_FILE :
            with open(JSON, "w", encoding="utf-8") as f:
                json.dump(self.poses, f, indent=4)

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
            self.save_json(CALIB_FILE)                     # ← 여기서 한 번만 저장
            for id in ids:
                c = self.calib[str(id)]
                print(f"{id}: {c['raw_min']} ~ {c['raw_max']}")

    def write_current_raw(self, id, key, value=None):
            
            if value is not None:
                self.calib[str(id)][key] = value
            else:
                self.calib[str(id)][key] = self.servo.ReadPos(id)[0]

            self.save_json(CALIB_FILE)

    def set_center(self, id):

        self.set_torque(id,1)

        _min = self.calib[str(id)]["raw_min"]
        _max = self.calib[str(id)]["raw_max"]

        mid = int(_min+(((_min-_max)**2)**0.5 * 0.5))

        print(f"{_min} - {_max} = {mid}")
        # raw, _, _ = self.servo.ReadPos(id)
        
        if self.read_pos(id) == mid:
            pass
        else:
            self.set_torque(id,1)

            self.servo.WritePosEx(id,mid,100,15)
            done = 0
            while done == 0:
                current_raw = self.servo.ReadPos(id)[0]
                print(current_raw)
                time.sleep(0.05)
                if abs(current_raw-mid) < 5:

                    diff = current_raw - int((POSITION_MAX+1)/2)

                    self.calib[str(id)]["raw_max"] -= diff
                    self.calib[str(id)]["raw_min"] -= diff

                    self.save_json(CALIB_FILE)

                    self.servo.write1ByteTxRx(id,ADDR_TORQUE_ENABLE,128)
                    print(f"now {id}'s middle is {self.read_pos(id)}")
                    self.set_torque(id,0)

                    return
                
    # move

    def move_to_degree(self, id, degree, speed = 500, acc = 15):

        clamped = {}

        c_raw, _, _ = self.servo.ReadPos(id)
        c_deg = raw2deg(c_raw)

        t_raw = deg2raw(degree)
        t_deg = degree

        if abs(t_deg - c_deg) < 1  :
            print("already here")
            return
        
        print(f"raw: {c_raw}, deg: {c_deg:.2f}")

        self.set_torque(id,1)
        
        mode, _, _ = self.servo.read1ByteTxRx(id,ADDR_MODE)
        if mode != 0:
            self.servo.write1ByteTxRx(id,ADDR_MODE,0)


        r_max,t_min = self.calib[str(id)]["raw_max"], self.calib[str(id)]["raw_min"]
        t_raw = max(t_min,min(r_max,t_raw))
        
        self.servo.WritePosEx(id, t_raw, speed, acc)

        clamped[str(id)] = raw2deg(t_raw)

        return clamped


    def move_to_sync(self,deg_dict, speed= 500, acc = 15):

        clamped = {}
        for id, degree in deg_dict.items():

            id = int(id)
            degree = float(degree)

            self.set_torque(id,1)
            raw = deg2raw(degree)

            #clamp min,max
            
            r_max,r_min = self.calib[str(id)]["raw_max"], self.calib[str(id)]["raw_min"]
            raw = max(r_min,min(r_max,raw))

            self.servo.SyncWritePosEx(id, raw, speed, acc)

            clamped[str(id)] = raw2deg(raw)
            
        self.servo.groupSyncWrite.txPacket()
        self.servo.groupSyncWrite.clearParam()

        return clamped

    def _wait_arrival(self, deg_dict, timeout=5):
        arrived_id = set()
        start = time.time()
        while True:
            goal = True
    
            for id, degree in deg_dict.items():
                
                id = int(id)
                degree = float(degree)
                
                t_deg = float(degree)
                c_deg = raw2deg(self.servo.ReadPos(id)[0])
                is_arrived = abs(t_deg-c_deg) < 2
                    
                load, _, _ = self.servo.read2ByteTxRx(id, ADDR_PRESENT_LOAD)
                curr, _, _ = self.servo.read2ByteTxRx(id, ADDR_PRESENT_CURRENT)
                temp, _, _ = self.servo.read2ByteTxRx(id, ADDR_PRESENT_TEMPERATURE)
    
                print(f"{id:<3} load: {load:<5} curr: {curr:<5} temp: {temp}")
    
                if is_arrived and id not in arrived_id:
    
                    arrived_id.add(id)
    
                goal &= is_arrived
            print('\n')
    
            if goal:
                break
    
            if time.time() - start > timeout:          # 함정4: stall이면 goal이 영영 안 돼 무한대기+과열
                stuck = [int(i) for i in deg_dict if int(i) not in arrived_id]
                print(f"!!timeout!! {timeout}s over — {stuck}, torque off")
                for id in deg_dict:                     # 진짜 stall일 수 있으니 토크 정리
                    self.set_torque(int(id), 0)
                break

            time.sleep(0.1)

    def get_current_pos(self):

        degree_dict = {}
        for id in IDS:
            raw, _, _ = self.servo.ReadPos(id)
            degree_dict[str(id)] = raw2deg(raw)
        return degree_dict

    def save_current_pose(self,pose_name):

        self.poses[pose_name] = self.get_current_pos() 
        self.save_json(POSES_FILE)