# silicone_control.py

import RPi.GPIO as GPIO
import time
import rospy
from shared_utils.scripts.arduino_communication import send_to_arduino

# === GPIO Setup ===
IN1, IN2, IN3, IN4 = 17, 18, 27, 22
GPIO.setmode(GPIO.BCM)
for pin in [IN1, IN2, IN3, IN4]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

# === 상태 ===
distance_cm = 999.0  # ROS 토픽으로 갱신됨
moved_steps = 0

# === 모터 설정 ===
CM_PER_STEP = 1.0 / 450.0
DEFAULT_SPEED_CM_S = 2.0
DIST_THRESHOLD_GO = 5.0
DIST_THRESHOLD_ABNORMAL = 10.0

# === 스텝 시퀀스 ===
SEQ = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

def distance_callback(msg):
    global distance_cm
    distance_cm = msg.data

def move_motor(steps, delay=0.005, direction=1):
    seq = SEQ if direction == 1 else SEQ[::-1]
    for _ in range(steps):
        for pattern in seq:
            GPIO.output(IN1, pattern[0])
            GPIO.output(IN2, pattern[1])
            GPIO.output(IN3, pattern[2])
            GPIO.output(IN4, pattern[3])
            time.sleep(delay)

def move_cm(cm, speed_cm_per_s=DEFAULT_SPEED_CM_S, direction=1):
    steps = int(abs(cm) / CM_PER_STEP)
    delay = max(0.002, 1.0 / (speed_cm_per_s * 450))
    move_motor(steps, delay=delay, direction=direction)
    return steps

def cleanup():
    for pin in [IN1, IN2, IN3, IN4]:
        GPIO.output(pin, 0)

# === GO ===
def go_mode():
    global moved_steps
    rospy.loginfo("GO: Moving forward until distance < 5cm")
    send_to_arduino("GO")
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))
    step = 1

    while not rospy.is_shutdown():
        if distance_cm < DIST_THRESHOLD_GO:
            rospy.loginfo("Distance < 5cm → STOP")
            break
        move_motor(step, delay=delay, direction=1)
        moved_steps += step

    send_to_arduino("STOP")
    cleanup()
    time.sleep(0.5)
    send_to_arduino("HIT")

# === ABNORMAL ===
def abnormal_mode():
    global moved_steps
    rospy.loginfo("ABNORMAL: Moving backward until distance > 10cm")
    send_to_arduino("ABNORMAL")
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))
    step = 1

    while not rospy.is_shutdown():
        if distance_cm > DIST_THRESHOLD_ABNORMAL:
            rospy.loginfo("Distance > 10cm → STOP")
            break
        move_motor(step, delay=delay, direction=-1)
        moved_steps -= step

    send_to_arduino("STOP")
    cleanup()
    send_to_arduino("1,F,2.0,5")
    send_to_arduino("2,F,30.0,90")
    time.sleep(5)

# === NORMAL ===
def normal_mode():
    global moved_steps
    rospy.loginfo(f"NORMAL: Returning {moved_steps} steps forward")
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))
    move_motor(moved_steps, delay=delay, direction=-1)
    moved_steps = 0
    rospy.loginfo("Return complete.")

# === QUIT ===
def quit_mode():
    rospy.loginfo("QUIT: Turning off all motors")
    send_to_arduino("QUIT")
    cleanup()
    GPIO.cleanup()
