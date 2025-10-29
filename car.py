#!/usr/bin/env python3
import can
import struct
import time
import os
import RPi.GPIO as GPIO
from gpiozero import AngularServo
from gpiozero.pins.lgpio import LGPIOFactory
from time import sleep
import zmq  # [추가] ZMQ 라이브러리

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

def control_servo(ang):
    """AS5600 각도에 따라 서보 제어"""
    if 0 <= ang <= 170:
        set_servo_angle(ang)
    elif 270 < ang < 360:
        set_servo_angle(10)
    elif 170 < ang < 270:
        set_servo_angle(170)
    else:
        set_servo_angle(90)

# ================================================================
# 3. CAN 통신 설정
# ================================================================
ANGLE_ID = 0x100
BUTTON_ID = 0x101
print(f"Listening for CAN messages on {channel}...")

try:
    bus = can.interface.Bus(channel=channel, bustype='socketcan')
    print("✅ CAN Bus connected successfully.\n")
except OSError:
    print(f"❌ Error: CAN interface '{channel}' not found or not up.")
    exit(1)

# ================================================================ 
# 3.5. ZMQ 설정 (Publisher) [추가] 
# ================================================================ 
print("📡 Setting up ZMQ Publisher...")
zmq_context = zmq.Context()
zmq_socket = zmq_context.socket(zmq.PUB)
zmq_port = 5556  # 사용할 포트 번호
zmq_socket.bind(f"tcp://*:{zmq_port}")
print(f"✅ ZMQ Publisher bound to tcp://*:{zmq_port}\n")

# ZMQ 토픽 (bytes) - Raw 데이터를 보낼 토픽
ZMQ_TOPIC_RAW_ANGLE = b"STEERING_ANGLE_RAW"

# ================================================================
# 4. 버튼 데이터 처리
# ================================================================
B2 = 1
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
        - BACKWARD 상태면 그대로 유지 (계속 후진)
    """
    global motor_state

    if len(data) == 0:
        print("⚠️ Received empty button data.")
        return

    btn = data[0]
    print(f"🛰️ Received button CAN data: {list(data)} (current state={motor_state})")

    # -------------------------------------------------
    # 전진 버튼 (B1)
    # -------------------------------------------------
    if btn == 0x01:
        if motor_state == "STOPPED":
            # 정지 상태에서 전진 시작
            forward(100)
            motor_state = "FORWARD"
            print("🟢 B1: FORWARD 시작 (forward 100%)")

        elif motor_state == "BACKWARD":
            # 후진 중일 때 전진 버튼을 누르면 후진 정지
            stop_motor()
            motor_state = "STOPPED"
            print("🟠 B1: 후진 정지 -> STOPPED")

        elif motor_state == "FORWARD":
            # 이미 전진 중이면 그대로
            print("🟢 B1: 이미 FORWARD 상태라 그대로 유지")

        else:
            print(f"❓ B1: 알 수 없는 상태 {motor_state}, 안전하게 정지")
            stop_motor()
            motor_state = "STOPPED"

    # -------------------------------------------------
    # 정지 버튼 (B2)
    # -------------------------------------------------
    elif btn == 0x02:
        if motor_state == "FORWARD":
            # 전진 중일 때는 즉시 정지
            stop_motor()
            motor_state = "STOPPED"
            print("🔴 B2: 즉시 정지 -> STOPPED")

        elif motor_state == "STOPPED":
            # 정지 상태에서 한 번 더 정지 버튼 → 후진
            backward(100)
            motor_state = "BACKWARD"
            print("🔁 B2: STOPPED -> BACKWARD (backward 100%)")

        elif motor_state == "BACKWARD":
            # 이미 후진 중이면 그대로 후진 유지
            print("🔁 B2: 이미 BACKWARD 상태 유지")

        else:
            print(f"❓ B2: 알 수 없는 상태 {motor_state}, 안전하게 정지")
            stop_motor()
            motor_state = "STOPPED"

    # -------------------------------------------------
    # 알 수 없는 버튼 값
    # -------------------------------------------------
    else:
        print(f"⚠️ Unknown button value: {btn}")


    


# ================================================================
# 5. 메인 루프 (ZMQ 전송 로직 포함)
# ================================================================
try:
    print("🚗 Waiting for CAN messages...\n")
    while True:
        msg = bus.recv(timeout=1.0)  # 1초 타임아웃
        if msg is None:
            # print("⏳ No CAN message received (waiting...)") # (로그가 너무 많아 주석 처리)
            continue

        # --- 모든 수신 메시지 로그 ---
        # print(f"📩 ID={hex(msg.arbitration_id)} | DLC={len(msg.data)} | Data={list(msg.data)}")

        # --- 각도 데이터 수신 ---
        if msg.arbitration_id == ANGLE_ID and len(msg.data) >= 2:
            
            # [변경] 원본 2바이트 raw data를 바로 사용
            raw_value_bytes = msg.data[0:2]
            
            # 로컬 서보 제어를 위해 정수 및 각도로 변환
            raw_value_int = struct.unpack('<H', raw_value_bytes)[0]
            angle_degrees = raw_value_int * (360.0 / 4096.0)
            
            # 1. 로컬 서보 제어
            control_servo(angle_degrees)
            print(f"🧭 Angle | Raw={raw_value_int:4d} | {angle_degrees:6.2f}°")

            # 2. [변경] ZMQ로 원본 2-byte 'raw data' 전송
            try:
                # message_payload를 CAN에서 받은 2바이트로 직접 사용
                message_payload = raw_value_bytes
                
                # [토픽, 실제 데이터] 형식으로 전송
                zmq_socket.send_multipart([ZMQ_TOPIC_RAW_ANGLE, message_payload])
                # print(f"🚀 ZMQ Sent Raw Bytes: {list(message_payload)}") # (디버깅용)

            except Exception as e:
                print(f"🔥 ZMQ Send Error: {e}")


        # --- 버튼 데이터 수신 ---
        elif msg.arbitration_id == BUTTON_ID:
            handle_button(msg.data)

        # --- 기타 데이터 ---
        # else:
        #     print(f"ℹ️ Unknown CAN ID: {hex(msg.arbitration_id)}") # (로그가 너무 많아 주석 처리)

except KeyboardInterrupt:
    print("\n🛑 Program stopped by user.")

except can.CanError as e:
    print(f"CAN bus error: {e}")

finally:
    # 모터 및 GPIO 정리
    stop_motor()
    motor_pwm.stop()
    GPIO.cleanup()
    
    # [추가] ZMQ 리소스 정리
    if 'zmq_socket' in locals():
        zmq_socket.close()
        print("ZMQ Socket closed.")
    if 'zmq_context' in locals():
        zmq_context.term()
        print("ZMQ Context terminated.")

    # CAN 버스 정리
    if 'bus' in locals() and bus is not None:
        bus.shutdown()
        print("CAN Bus shut down.")
    os.system(f"sudo ip link set {channel} down")
    print(f"🧹 CAN interface '{channel}' turned off.")