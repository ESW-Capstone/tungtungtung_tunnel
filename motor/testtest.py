# GO -> 팔이 앞으로, 초음파 센서로 멈추기. 솔레노이드 타격 및 정상 / 비정상 판별. 이게 3번 코드

import serial
import time
import threading

arduino2 = serial.Serial('COM11', 9600)
time.sleep(2)

def send_to_arduino2(msg):
    arduino2.write((msg + '\n').encode())
    print(f"[TX → Arduino2] {msg}")

def listen_from_arduino2():
    while True:
        if arduino2.in_waiting:
            line = arduino2.readline().decode().strip()
            print(f"[RX from Arduino2] {line}")
            if line == "STOP":
                print(" → [ACTION] 슬라이드 이동 완료")
            elif line == "HIT_READY":
                print("녹음준비")
            elif line == "CHECK":
                time.sleep(2)
                print("녹음분석")
            elif line == "READY":
                print(" → [ACTION] 보정 완료, 모터 명령 실행")
                send_to_arduino2("1,F,2.0,5")   #모터id 1(r축), F:정방향/B:역방향, 이동속도, 이동거리(cm)
                send_to_arduino2("2,F,30.0,90") #모터id 2(theta축), F:정방향/B:역방향, 이동속도, 이동거리(각도)
                time.sleep(10)
                send_to_arduino2("NORMAL")
        time.sleep(0.5)

def handle_user_input():
    print("명령어 입력: go / abnormal / normal / quit")
    while True:
        cmd = input("\n>> 명령 입력: ").strip().lower()
        if cmd == "go":
            send_to_arduino2("GO")
        elif cmd == "abnormal":
            send_to_arduino2("ABNORMAL")
        elif cmd == "normal":
            send_to_arduino2("NORMAL")
        elif cmd == "quit":
            print(" → 종료합니다.")
            break
        else:
            print("지원하지 않는 명령입니다.")

threading.Thread(target=listen_from_arduino2, daemon=True).start()
handle_user_input()
