# img-collector.py
# -*- coding: utf-8 -*-
# =============================================================================
# Description : 차량 주행(DC/Servo)과 웹캠 영상 촬영을 동시에 수행하는
#               데이터 수집 통합 실행 스크립트.
#
# Author : Youngchul Jung (Modified by Gemini for data-balancing)
# =============================================================================

import threading
import subprocess
import time

# 가정: camera_capture.py 및 drive.py 임포트 경로가 정상 작동한다고 가정합니다.
# 실제 코드를 실행하려면 이 두 파일이 유효해야 합니다.
try:
    # hw_control/drive.py 모듈 임포트
    import hw_control.drive as drive
    # camera/camera_capture.py 모듈 임포트 (이 함수 내부에 저장 로직이 있다고 가정)
    from camera.camera_capture import camera_capture_loop 
except ImportError as e:
    # 에러 발생 시 처리 (환경 설정 문제)
    print(f"Error importing modules: {e}. Ensure you are running from the correct directory.")
    sys.exit(1)


# -----------------------------------------------------------------------------
# 공통 상태 / 설정값
# -----------------------------------------------------------------------------
# stop_flag을 리스트로 둔 이유: 스레드 간 참조 공유를 위함
stop_flag = [False]

# 데이터 저장 디렉토리 및 파일 경로
OUTPUT_DIR = "dataset"
CSV_FILE = "dataset/data_labels.csv"

# 촬영 해상도 및 프레임 저장 주기
IMAGE_W, IMAGE_H = 640, 480
SAVE_INTERVAL = 0.5  # 초 단위 (0.5초 = 초당 2프레임 저장)


# -----------------------------------------------------------------------------
# 현재 주행 상태 조회 함수 (★ drive.py의 수정 사항 반영)
# -----------------------------------------------------------------------------
def get_state():
    """
    camera_capture_loop에서 라벨 저장 시 사용하는 콜백 함수.
    drive 모듈 내부의 현재 각도, 속도, 그리고 ★조작 활성화 상태를 반환한다.
    반환: (servo_angle, motor_speed, is_active_control)
    """
    # drive.py의 get_current_state()는 이제 3개의 값을 반환함
    angle, speed, is_active = drive.get_current_state()
    return angle, speed, is_active


# -----------------------------------------------------------------------------
# 메인 실행부
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    print(f"[INFO] Data Collector Started (Save Interval: {SAVE_INTERVAL}s)")
    
    try:
        # ---------------------------------------------------------------------
        # 1) 주행 제어 스레드
        # ---------------------------------------------------------------------
        drive_thread = threading.Thread(
            target=drive.run_drive_control,
            args=(stop_flag,),
            daemon=True,
        )

        # ---------------------------------------------------------------------
        # 2) 카메라 캡처 스레드
        #    ★ camera_capture_loop에 get_state 콜백을 넘겨, 내부에서 is_active_control을 확인하도록 함
        # ---------------------------------------------------------------------
        camera_thread = threading.Thread(
            target=camera_capture_loop,
            args=(
                OUTPUT_DIR,
                CSV_FILE,
                IMAGE_W, IMAGE_H,
                SAVE_INTERVAL,
                stop_flag,
                get_state,  # ★ 3개의 값(각도, 속도, is_active)을 반환하는 콜백 함수 전달
            ),
            daemon=True,
        )

        # 스레드 시작
        drive_thread.start()
        camera_thread.start()
        print("[INFO] Threads started. Press ESC or Ctrl+C to stop.")

        # 두 스레드 종료까지 대기
        # (Ctrl+C 발생 전까지 메인 스레드가 살아있도록 대기)
        drive_thread.join()
        camera_thread.join()
        
    except KeyboardInterrupt:
        # Ctrl+C 를 누르면 두 스레드 종료 요청
        print("\n[INFO] Interrupted by user. Sending stop signal...")
        stop_flag[0] = True
        
        # 스레드가 종료될 때까지 잠시 대기
        time.sleep(1) 
        
    finally:
        # drive.py의 cleanup은 drive_thread.join() 후 drive.py의 finally 블록에서 자동 처리됩니다.
        print("[INFO] Data collection finished.")
