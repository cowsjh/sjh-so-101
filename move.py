import sys, time
from src.servo import Arm


def sync_write(arm : Arm, pose_name, speed=500, acc=15):
    """poses.json에 저장된 이름 자세로 6축 동시 이동 → 도착까지 대기."""
    if pose_name not in arm.poses:
        print(f"'{pose_name}' 자세 없음. 가능: {list(arm.poses)}")
        return
    print(f"\n▶ {pose_name}")
    target = arm.move_to_sync(arm.poses[pose_name], speed, acc)  # 쏘고 clamp된 목표를 돌려받음
    arm._wait_arrival(target, timeout = 100)                                    # 잘린 목표로 판정해야 무한대기 안 걸림

def move(arn:Arm, id, deg):
    target = arm.move_to_degree(id,deg)
    arm._wait_arrival(target, timeout = 10)


if __name__ == "__main__":
    
    sequence = sys.argv[1:] or ["home", "activate", "home"]
    
    with Arm() as arm:
        for name in sequence:
            sync_write(arm, name)
            time.sleep(0.5)   
        # move(arm,int(sys.argv[1]),int(sys.argv[2]))
