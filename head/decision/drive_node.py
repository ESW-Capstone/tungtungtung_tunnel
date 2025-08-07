#!/usr/bin/env python3
import rospy
import serial
import threading
import time
from std_msgs.msg import Bool
import sys
import termios
import tty

# === Serial Setup ===
ser = serial.Serial('/dev/ttyTHS1', 9600, timeout=1)
time.sleep(2)  # 시리얼 안정화

# === 글로벌 상태 ===
manual_mode = False
thermal_triggered = False
lock = threading.Lock()

# === 키 입력 처리 ===
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def keyboard_listener():
    global manual_mode
    while not rospy.is_shutdown():
        key = get_key()
        with lock:
            manual_mode = True  # 수동 모드 진입
        if key.upper() in ['W', 'A', 'S', 'D', 'Q']:
            send_serial(key.upper())
            rospy.loginfo(f"[MANUAL] Sent: {key.upper()}")
        elif key == '\x03':  # Ctrl-C
            break

# === Serial 통신 전송 ===
def send_serial(cmd):
    ser.write((cmd + '\n').encode())

# === Thermal 콜백 ===
def thermal_callback(msg):
    global thermal_triggered
    if msg.data:
        with lock:
            thermal_triggered = True
            send_serial('Q')  # 즉시 정지
            rospy.logwarn("[THERMAL] Detected! Emergency Stop Triggered.")

# === 서보모터 제어 ===
def move_servo():
    for angle in range(0, 31):
        send_serial(f'{angle}')
        rospy.loginfo(f"[AUTO] Servo Angle: {angle}")
        time.sleep(0.05)

# === 차 이동 제어 ===
def move_car_forward():
    send_serial('W')
    rospy.loginfo("[AUTO] Car moving forward")
    time.sleep(2)
    send_serial('Q')
    rospy.loginfo("[AUTO] Car stopped")

# === 정상 동작 루프 ===
def auto_mode_loop():
    global manual_mode, thermal_triggered
    rate = rospy.Rate(0.5)  # 0.5Hz = 2초에 1번 반복
    while not rospy.is_shutdown():
        with lock:
            if manual_mode:
                continue  # 수동 모드면 자동 중단
            if thermal_triggered:
                continue  # thermal 신호 수신 중이면 중단

        move_servo()
        move_car_forward()

        with lock:
            if thermal_triggered:
                continue
            move_servo()

        rate.sleep()

# === 메인 ===
def main():
    rospy.init_node('drive_node')
    rospy.Subscriber('/thermal_flag_mode0', Bool, thermal_callback)

    # 스레드 시작
    threading.Thread(target=keyboard_listener, daemon=True).start()
    threading.Thread(target=auto_mode_loop, daemon=True).start()

    rospy.spin()

if __name__ == '__main__':
    main()
