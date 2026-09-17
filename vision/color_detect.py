"""색 물체 검출 → 중심점(centroid) 추출.
SO-101 카메라 로드맵 2번. 단독 스크립트(ROS 아님), HSV 색검출 학습용.
실행: python3 color_detect.py  (종료: 창에서 q)
"""
import cv2
import numpy as np

CAM_INDEX = 2  # Innomaker U20CAM = /dev/video2 (v4l2-ctl --list-devices 로 확인)

# --- 검출할 색 범위 (HSV) ---
# 파랑 물체 기준 예시값. 네 물체/조명에 맞춰 튜닝해야 함.
# OpenCV HSV 범위: H 0~179, S 0~255, V 0~255 (H가 360도 아니라 180 스케일인 점 주의)
LOWER = np.array([100, 80, 80])
UPPER = np.array([130, 255, 255])

MIN_AREA = 500  # 이보다 작은 덩어리는 노이즈로 무시 (픽셀 면적)


def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"카메라 {CAM_INDEX} 못 엶")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        # 1) BGR → HSV. 색상(H)이 밝기(V)와 분리돼 조명 변화에 강함.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2) 범위 안 픽셀만 흰색(255), 나머지 검정(0)인 이진 마스크.
        mask = cv2.inRange(hsv, LOWER, UPPER)

        # 3) 잔노이즈 제거: open(침식→팽창)으로 작은 흰점 날림. (없어도 동작, 품질 개선용)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 4) 컨투어(흰 덩어리 외곽선) 찾기.
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 가장 큰 덩어리 = 목표 물체로 가정.
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) >= MIN_AREA:
                # 5) 모멘트로 중심(cx, cy) 계산. m00=면적, m10/m01=1차 모멘트.
                M = cv2.moments(c)
                if M["m00"] > 0:  # 0 나눗셈 가드
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.putText(frame, f"({cx},{cy})", (cx + 10, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow("frame", frame)
        cv2.imshow("mask", mask)  # 마스크 같이 보며 HSV 범위 튜닝
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
