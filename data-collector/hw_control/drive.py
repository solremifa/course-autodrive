# drive.py
# -*- coding: utf-8 -*-
"""
===============================================================================
File Name       : drive.py
Description     : Jetson Nano + L298N(DC Motor) + MG996R Servo Control
                  - DC Motor : L298N (PWM pin 33)
                  - Servo    : MG996R (PWM pin 32)
                  - Keyboard input: input_utils.get_key_nonblock()

Author          : Youngchul Jung (Modified by Gemini for venv and data-balancing)
Date Created    : 2025-11-13 (Last Modified: 2025-11-26)
===============================================================================
"""
import time
import Jetson.GPIO as GPIO
import subprocess
import getpass
import sys # sys 모듈 추가 (터미널 설정 관련)

try:
    # 패키지로 임포트되는 경우 (img-collector 에서 사용)
    from hw_control.input_utils import get_key_nonblock
except ImportError:
    # drive.py 를 단독 실행할 때
    from input_utils import get_key_nonblock


# ================= PWM ENABLE ==================
def activate_jetson_pwm(auto_install_busybox=True):
    """
    Jetson Nano에서 특정 핀을 PWM 기능으로 사용하기 위해
    레지스터(devmem)를 직접 설정하는 함수
    """
    # sudo 권한이 필요하므로 실행 시 비밀번호를 한 번 입력받음
    sudo_pw = getpass.getpass("Enter sudo password: ")

    def run_sudo(cmd):
        # 입력받은 sudo 비밀번호를 사용해 root 권한 명령 실행
        full_cmd = f"echo {sudo_pw} | sudo -S {cmd}"
        subprocess.run(full_cmd, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # busybox(devmem 포함) 자동 설치 옵션
    if auto_install_busybox:
        try:
            subprocess.run(
                "busybox --help",
                shell=True,
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except subprocess.CalledProcessError:
            # busybox가 설치되어 있지 않으면 설치
            run_sudo("apt update && apt install -y busybox")

    # Jetson 특정 레지스터를 devmem으로 직접 설정하여
    # 해당 핀들을 PWM 모드로 매핑
    cmds = [
        "busybox devmem 0x700031fc 32 0x45",
        "busybox devmem 0x6000d504 32 0x2",
        "busybox devmem 0x70003248 32 0x46",
        "busybox devmem 0x6000d100 32 0x00",
    ]
    for c in cmds:
        run_sudo(c)


# ================= CONSTANTS ====================
# Jetson BOARD 핀 번호
MOTOR_PWM_PIN         = 33    # DC 모터 PWM 핀
MOTOR_DIRECTION_PIN1  = 29    # L298N IN2
MOTOR_DIRECTION_PIN2  = 31    # L298N IN1
SERVO_PWM_PIN         = 32    # 서보 PWM 핀

# PWM 설정 값
MOTOR_PWM_FREQUENCY   = 1000  # DC 모터 PWM 주파수 (Hz)
SERVO_PWM_FREQUENCY   = 50    # 서보 PWM 주파수 (Hz)

# 서보 제어 범위 (듀티 비율)
SERVO_MIN_DC = 4.5             # 듀티 범위 확장 (4.5%~10.5% - 서보에 따라 튜닝 필요)
SERVO_MAX_DC = 10.5            # 듀티 범위 확장
# ★ 수정됨: 9개 클래스로 변경 (15도 간격)
SERVO_STEPS  = [30, 45, 60, 75, 90, 105, 120, 135, 150] 
SERVO_INDEX  = 4               # 초기 인덱스 (90도, 중앙)
 
# 모터 속도 및 제어 관련 상수
motor_speed       = 60        # 기본 모터 속도 (0~100)
MOTOR_STEP      = 10          # A/Z 키로 속도 변경 단위
KEY_DELAY       = 0.04        # 키 입력 폴링 주기 (초)

# 정지 시 감속 설정
STOP_DECAY_STEP  = 3
STOP_DECAY_DELAY = 0.015

# 모터 방향 상태 및 데이터 수집 플래그
current_direction = None
is_active_control = False  # <-- ★ 데이터 수집 활성화 플래그 (추가됨)


# ================= GPIO INIT ====================
# BOARD 모드 사용 (실제 보드 핀 번호 기준)
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# 핀 모드 설정
GPIO.setup(MOTOR_PWM_PIN, GPIO.OUT)
GPIO.setup(MOTOR_DIRECTION_PIN1, GPIO.OUT)
GPIO.setup(MOTOR_DIRECTION_PIN2, GPIO.OUT)
GPIO.setup(SERVO_PWM_PIN, GPIO.OUT)

# PWM 객체 생성
motor_pwm = GPIO.PWM(MOTOR_PWM_PIN, MOTOR_PWM_FREQUENCY)
servo_pwm = GPIO.PWM(SERVO_PWM_PIN, SERVO_PWM_FREQUENCY)

# 초기 PWM 시작 (모터는 정지, 서보는 대략 중앙 90도)
initial_duty = SERVO_MIN_DC + (90 / 180.0) * (SERVO_MAX_DC - SERVO_MIN_DC)
motor_pwm.start(0)
servo_pwm.start(initial_duty)


# ================= SERVO ========================
def angle_to_duty(angle):
    """
    서보 각도(0~180도)를 PWM 듀티(%)로 변환
    """
    # 보호: 입력 각도를 0~180 범위로 클램핑
    angle = max(0, min(180, angle))
    return SERVO_MIN_DC + (angle / 180.0) * (SERVO_MAX_DC - SERVO_MIN_DC)


def set_servo_angle(angle):
    """
    서보를 특정 각도(도 단위)로 회전시키는 함수
    """
    duty = angle_to_duty(angle)
    servo_pwm.ChangeDutyCycle(duty)
    # 서보가 움직일 시간을 조금 부여 (필요 시 튜닝 가능)
    time.sleep(0.005)


# ================= MOTOR ========================
def control_motor(direction):
    """
    DC 모터를 지정한 방향으로 구동하고, 활동 플래그를 ON으로 설정
    """
    global current_direction, is_active_control

    if direction == "forward":
        # IN1=LOW, IN2=HIGH → 전진
        GPIO.output(MOTOR_DIRECTION_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_DIRECTION_PIN2, GPIO.HIGH)

    elif direction == "backward":
        # IN1=HIGH, IN2=LOW → 후진
        GPIO.output(MOTOR_DIRECTION_PIN1, GPIO.HIGH)
        GPIO.output(MOTOR_DIRECTION_PIN2, GPIO.LOW)

    # 설정된 속도로 PWM 출력
    motor_pwm.ChangeDutyCycle(motor_speed)
    # 현재 방향 상태 기록
    current_direction = direction
    is_active_control = True # <-- ★ 모터 작동 시 플래그 ON


def smooth_stop():
    """
    모터를 서서히 줄이면서 정지하고, 활동 플래그를 OFF로 설정
    """
    global current_direction, is_active_control

    # motor_speed에서 0까지 STOP_DECAY_STEP만큼 감소시키며 듀티 변경
    for sp in range(motor_speed, -1, -STOP_DECAY_STEP):
        motor_pwm.ChangeDutyCycle(sp)
        time.sleep(STOP_DECAY_DELAY)

    # 완전 정지 상태
    motor_pwm.ChangeDutyCycle(0)
    GPIO.output(MOTOR_DIRECTION_PIN1, GPIO.LOW)
    GPIO.output(MOTOR_DIRECTION_PIN2, GPIO.LOW)

    # 방향 상태 및 활동 상태 초기화
    current_direction = None
    is_active_control = False # <-- ★ 완전 정지 시 플래그 OFF


def get_current_state():
    """
    img-collector에서 현재 상태 조회 시, is_active_control 플래그를 포함하여 반환
    반환: (servo_angle_deg, motor_speed_percent, is_active_control)
    """
    global SERVO_INDEX, SERVO_STEPS, motor_speed, is_active_control
    # ★ 반환 값에 is_active_control 플래그 추가
    return SERVO_STEPS[SERVO_INDEX], motor_speed, is_active_control 


def run_drive_control(stop_flag=None):
    """
    키보드 입력을 받아 모터/서보를 제어하는 메인 루프.
    """
    global current_direction, SERVO_INDEX, motor_speed, is_active_control

    activate_jetson_pwm()
    set_servo_angle(SERVO_STEPS[SERVO_INDEX])

    print(
        "\n=== CONTROL MODE ===\n"
        "  ↑ : Forward\n"
        "  ↓ : Backward\n"
        "  ← / → : Steering Left / Right (15 deg step)\n" # ★ 스텝 변경 반영
        "  S : Center (90°)\n"
        "  A/Z : Speed + / -\n"
        "  ESC / Ctrl+C : Exit\n"
    )

    try:
        while True:
            # 외부 플래그로 종료 요청 시
            if stop_flag is not None and stop_flag[0]:
                break

            # 1. 키 입력 확인
            key = get_key_nonblock()

            # 키 입력이 있을 때 (조작 발생)
            if key is not None:

                if key == "UP":
                    if current_direction == "backward":
                        smooth_stop()
                    else:
                        control_motor("forward")

                elif key == "DOWN":
                    if current_direction == "forward":
                        smooth_stop()
                    else:
                        control_motor("backward")

                elif key == "LEFT":
                    SERVO_INDEX = max(0, SERVO_INDEX - 1)
                    set_servo_angle(SERVO_STEPS[SERVO_INDEX])
                    is_active_control = True # <-- ★ 조향 시 플래그 ON

                elif key == "RIGHT":
                    SERVO_INDEX = min(len(SERVO_STEPS)-1, SERVO_INDEX + 1)
                    set_servo_angle(SERVO_STEPS[SERVO_INDEX])
                    is_active_control = True # <-- ★ 조향 시 플래그 ON

                elif key in ("s", "S"):
                    SERVO_INDEX = SERVO_STEPS.index(90)
                    set_servo_angle(90)
                    is_active_control = True # <-- ★ 중앙 복귀(S) 시 플래그 ON

                elif key in ("a", "A"):
                    motor_speed = min(100, motor_speed + MOTOR_STEP)
                    print("[MOTOR] speed:", motor_speed)
                    
                    if current_direction is not None:
                        motor_pwm.ChangeDutyCycle(motor_speed)
                    is_active_control = True # <-- ★ 속도 변경 시 플래그 ON

                elif key in ("z", "Z"):
                    motor_speed = max(0, motor_speed - MOTOR_STEP)
                    print("[MOTOR] speed:", motor_speed)
                    
                    if current_direction is not None:
                        motor_pwm.ChangeDutyCycle(motor_speed)
                    is_active_control = True # <-- ★ 속도 변경 시 플래그 ON
                    
                elif key in ("t", "T"):
                    smooth_stop()
                    
                elif key in ("ESC", "CTRL_C"):
                    break
            
            # 키 입력이 없을 때 (조작 중지 상태)
            else:
                # 모터도 멈추고 키 입력도 없으면 활동 상태 OFF (촬영 중지)
                if current_direction is None and is_active_control:
                     is_active_control = False
                     
            time.sleep(KEY_DELAY)

    finally:
        smooth_stop()
        motor_pwm.stop()
        servo_pwm.stop()
        GPIO.cleanup()
        print("GPIO cleaned up.")

# ================= MAIN =========================
if __name__ == "__main__":
    print("[DEBUG] Simple Controller Started")
    # 단독 실행 시
    run_drive_control(stop_flag=None)
