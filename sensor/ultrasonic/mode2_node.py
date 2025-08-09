#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32
import threading
import time

from silicone_control import (
    go_mode, abnormal_mode, normal_mode, quit_mode, distance_callback
)

# === 상태 변수 ===
manual_command = None
manual_lock = threading.Lock()

# === 수동 명령 입력 쓰레드 ===
def manual_input_thread():
    global manual_command
    while not rospy.is_shutdown():
        cmd = input("\n[Manual] Command (go / abnormal / normal / quit): ").strip().lower()
        if cmd in ["go", "abnormal", "normal", "quit"]:
            with manual_lock:
                manual_command = cmd
        else:
            print("Invalid command")

# === 수동 명령 감지 및 실행 ===
def handle_command(pub):
    global manual_command
    with manual_lock:
        cmd = manual_command
        manual_command = None  # 실행 후 초기화

    if cmd == "go":
        rospy.loginfo("[Manual Override] → GO")
        go_mode()
    elif cmd == "abnormal":
        rospy.loginfo("[Manual Override] → ABNORMAL")
        abnormal_mode()
        pub.publish("done")
        rospy.loginfo("[mode2] (manual) 'done' 메시지 전송 완료")
    elif cmd == "normal":
        rospy.loginfo("[Manual Override] → NORMAL")
        normal_mode()
        pub.publish("done")
        rospy.loginfo("[mode2] (manual) 'done' 메시지 전송 완료")
    elif cmd == "quit":
        rospy.loginfo("[Manual Override] → QUIT")
        quit_mode()

# === 메인 자동 제어 ===
def main():
    global manual_command

    rospy.init_node("mode2_node")
    rospy.Subscriber('/distance_cm', Float32, distance_callback)

    pub = rospy.Publisher('/mode_result', String, queue_size=10)
    rospy.sleep(1.0)

    # 수동 명령 입력 쓰레드 시작
    threading.Thread(target=manual_input_thread, daemon=True).start()

    # === 자동 흐름 1단계: go_mode ===
    rospy.loginfo("AUTO: go_mode 시작")
    go_mode()

    # === 자동 흐름 2단계: abnormal_mode 대기 (실시간 수동 개입 가능) ===
    rospy.loginfo("AUTO: abnormal_mode 대기 중 (10초 내 수동 개입 가능)")
    total_duration = 5  # 초
    start_time = time.time()
    manual_executed = False

    while time.time() - start_time < total_duration and not rospy.is_shutdown():
        if manual_command:
            handle_command(pub)
            manual_executed = True
            break
        time.sleep(0.1)

    # 수동 명령이 없었으면 자동으로 abnormal 실행
    if not manual_executed and not rospy.is_shutdown():
        rospy.loginfo("AUTO: abnormal_mode 실행")
        abnormal_mode()
        pub.publish("done")
        rospy.loginfo("[mode2] (auto) 'done' 메시지 전송 완료")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("중단됨")
