#!/usr/bin/env python3
import can
import struct
import time
import os
import RPi.GPIO as GPIO
from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory
from time import sleep
import zmq

# ================================================================
# 0. CAN 인터페이스 자동 설정
# ================================================================
channel = 'can0'
bitrate = 200000

print(f"🛠️ Setting up CAN interface '{channel}' with bitrate {bitrate}...")
os.system(f"sudo ip link set {channel} down > /dev/null 2>&1")
os.system(f"sudo ip link set {channel} type can bitrate {bitrate}")
os.system(f"sudo ip link set {channel} up")
print(f"✅ CAN interface '{channel}' is now up and ready.\n")

# ================================================================
# 1. DC 모터 제어 핀 설정 (HW095 모듈)
# ================================================================
ENA = 18    # PWM 핀
IN1 = 17    # 방향 제어 1
IN2 = 27    # 방향 제어 2

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

motor_pwm = GPIO.PWM(ENA, 100)
motor_pwm.start(0)

def stop_motor():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(0)
    print("🟥 Motor stopped")

def forward(speed=80):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    motor_pwm.ChangeDutyCycle(speed)
    print(f"🟩 Motor forward ({speed}%)")

def backward(speed=80):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    motor_pwm.ChangeDutyCycle(speed)
    print(f"🟦 Motor backward ({speed}%)")

# ================================================================
# 2. 서보 초기화 (LGPIOFactory 사용 → RPi.GPIO PWM과 완전 분리)
# ================================================================
factory = LGPIOFactory()
servo = AngularServo(
    14,
    min_angle=0,
    max_angle=180,
    min_pulse_width=1/1000,
    max_pulse_width=2/1000,
    pin_factory=factory
)
print("🧭 Servo initialized on GPIO14 (using LGPIOFactory)")

def set_servo_angle(angle):
    angle = max(0, min(angle, 180))
    servo.angle = angle
    sleep(0.02)

def control_servo(ang_deg):
    """
    AS5600 각도(0~360-ish)를 받아서 실제 서보 목표각으로 보냄.
    이 부분은 기존 로직 그대로 사용.
    """
    if 0 <= ang_deg <= 170:
        set_servo_angle(ang_deg)
    elif 270 < ang_deg < 360:
        # wrap-around 보호용
        set_servo_angle(10)
    elif 170 < ang_deg < 270:
        set_servo_angle(170)
    else:
        set_servo_angle(90)

# ================================================================
# 3. CAN 통신 설정
# ================================================================
ANGLE_ID  = 0x100
BUTTON_ID = 0x101
print(f"Listening for CAN messages on {channel}...")

try:
    bus = can.interface.Bus(channel=channel, bustype='socketcan')
    print("✅ CAN Bus connected successfully.\n")
except OSError:
    print(f"❌ Error: CAN interface '{channel}' not found or not up.")
    exit(1)

# ================================================================
# 3.5. ZMQ 설정 (Jetson0으로 조향각 PUSH)
# ================================================================
print("📡 Setting up ZMQ PUSH...")
zmq_context = zmq.Context()
zmq_socket = zmq_context.socket(zmq.PUSH)

# Jetson0 IP (Jetson0이 PULL.bind 한 쪽)
jetson0_ip = "163.180.179.240"
zmq_port  = 5556

# 라즈베리파이는 connect 쪽
zmq_socket.connect(f"tcp://{jetson0_ip}:{zmq_port}")
print(f"✅ ZMQ PUSH connected to tcp://{jetson0_ip}:{zmq_port}\n")

# ================================================================
# 4. 버튼 데이터 처리
# ================================================================
motor_state = "STOPPED"  # 가능한 값: "FORWARD", "STOPPED", "BACKWARD"

def handle_button(data):
    """
    B1 (0x01): 전진 버튼
        - STOPPED 상태면 forward()로 전진 시작 → FORWARD 상태
        - BACKWARD 상태면 후진을 멈추고 stop_motor() → STOPPED 상태
        - FORWARD 상태면 그대로 유지

    B2 (0x02): 정지 버튼
        - FORWARD 상태면 즉시 stop_motor() → STOPPED 상태
        - STOPPED 상태면 backward()로 후진 시작 → BACKWARD 상태
        - BACKWARD 상태면 그대로 유지
    """
    global motor_state

    if len(data) == 0:
        print("⚠️ Received empty button data.")
        return

    btn = data[0]
    print(f"🛰️ Received button CAN data: {list(data)} (current state={motor_state})")

    if btn == 0x01:
        # 전진 버튼
        if motor_state == "STOPPED":
            forward(100)
            motor_state = "FORWARD"
            print("🟢 B1: FORWARD 시작 (forward 100%)")
        elif motor_state == "BACKWARD":
            stop_motor()
            motor_state = "STOPPED"
            print("🟠 B1: 후진 정지 -> STOPPED")
        elif motor_state == "FORWARD":
            print("🟢 B1: 이미 FORWARD 상태라 그대로 유지")
        else:
            print(f"❓ B1: 알 수 없는 상태 {motor_state}, 안전하게 정지")
            stop_motor()
            motor_state = "STOPPED"

    elif btn == 0x02:
        # 정지/후진 버튼
        if motor_state == "FORWARD":
            stop_motor()
            motor_state = "STOPPED"
            print("🔴 B2: 즉시 정지 -> STOPPED")
        elif motor_state == "STOPPED":
            backward(100)
            motor_state = "BACKWARD"
            print("🔁 B2: STOPPED -> BACKWARD (backward 100%)")
        elif motor_state == "BACKWARD":
            print("🔁 B2: 이미 BACKWARD 상태 유지")
        else:
            print(f"❓ B2: 알 수 없는 상태 {motor_state}, 안전하게 정지")
            stop_motor()
            motor_state = "STOPPED"
    else:
        print(f"⚠️ Unknown button value: {btn}")


# ================================================================
# 5. 메인 루프
#   - CAN에서 조향각(raw) 읽음
#   - 로컬 서보 제어
#   - Jetson0으로 현재 조향각(클램프된 0~180) PUSH
# ================================================================
try:
    print("🚗 Waiting for CAN messages...\n")
    while True:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            continue

        # 조향 각도 메시지
        if msg.arbitration_id == ANGLE_ID and len(msg.data) >= 2:
            # (1) CAN으로부터 2byte raw angle 읽기
            raw_value_bytes = msg.data[0:2]  # little-endian 16bit
            raw_value_int = struct.unpack('<H', raw_value_bytes)[0]

            # (2) 12-bit AS5600 값 -> 실제 각도(0~360도 스케일)
            angle_degrees = raw_value_int * (360.0 / 4096.0)

            # (3) 로컬 서보 구동
            control_servo(angle_degrees)

            # (4) Jetson0에 보낼 "엔코더 기반 조향각"
            #     Jetson0은 0~180이 들어온다고 가정하고,
            #     그걸 10~170도로 매핑해서 HUD용/미래 궤적용으로 사용.
            encoder_angle_clamped = max(0.0, min(180.0, angle_degrees))

            # (5) ZMQ로 PUSH (JSON 한 덩어리)
            try:
                zmq_socket.send_json({
                    "encoder_deg": float(encoder_angle_clamped)
                })
            except Exception as e:
                print(f"🔥 ZMQ Send Error: {e}")

            # (6) 로깅
            print(
                f"🧭 Angle | raw_int={raw_value_int:4d} | "
                f"angle={angle_degrees:6.2f}° | tx={encoder_angle_clamped:6.2f}°"
            )

        # 버튼 메시지
        elif msg.arbitration_id == BUTTON_ID:
            handle_button(msg.data)

        # 그 외 ID는 무시/확장 포인트

except KeyboardInterrupt:
    print("\n🛑 Program stopped by user.")

except can.CanError as e:
    print(f"CAN bus error: {e}")

finally:
    # 모터 및 GPIO 정리
    stop_motor()
    motor_pwm.stop()
    GPIO.cleanup()

    # ZMQ 정리
    try:
        zmq_socket.close()
        print("ZMQ Socket closed.")
    except Exception:
        pass
    try:
        zmq_context.term()
        print("ZMQ Context terminated.")
    except Exception:
        pass

    # CAN 버스 정리
    if 'bus' in locals() and bus is not None:
        bus.shutdown()
        print("CAN Bus shut down.")

    os.system(f"sudo ip link set {channel} down")
    print(f"🧹 CAN interface '{channel}' turned off.")
