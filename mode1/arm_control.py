import Jetson.GPIO as GPIO
import time
import rospy
import serial
import threading

from angle_calculate import listen_from_arduino
from std_msgs.msg import Int32

# === GPIO Setup ===
IN1, IN2, IN3, IN4 = 17, 18, 27, 22

_GPIO_INITED = False
_GPIO_LOCK = threading.Lock()

def _ensure_gpio_ready():
    global _GPIO_INITED
    if _GPIO_INITED:
        return
    with _GPIO_LOCK:
        if _GPIO_INITED:
            return
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in [IN1, IN2, IN3, IN4]:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        _GPIO_INITED = True

def cleanup_gpio():
    global _GPIO_INITED
    if not _GPIO_INITED:
        return
    with _GPIO_LOCK:
        try:
            for pin in [IN1, IN2, IN3, IN4]:
                try:
                    GPIO.output(pin, 0)
                except Exception:
                    pass
            GPIO.cleanup([IN1, IN2, IN3, IN4])
        except Exception:
            pass
        _GPIO_INITED = False

# === Persistent serial ===
class ArduinoLink:
    def __init__(self, port='/dev/ttyACM0', baud=9600, timeout=1):
        self._lock = threading.Lock()
        self._ser = serial.Serial(port, baud, timeout=timeout)

    def send(self, msg: str):
        with self._lock:
            try:
                self._ser.write((msg + "\n").encode())
                rospy.loginfo(f"[TX->Arduino] {msg}")
            except serial.SerialException as e:
                rospy.logerr(f"Serial error: {e}")

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

# === Motion / thresholds ===
moved_steps = 0

CM_PER_STEP = 1.0 / 450.0
DEFAULT_SPEED_CM_S = 2.0
DIST_THRESHOLD_GO = 5.0
DIST_THRESHOLD_ABNORMAL = 10.0

SEQ = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1],
]

# === Globals initialized at runtime ===
arduino = None
step_pub = None

def send_to_arduino(msg: str):
    if arduino is not None:
        arduino.send(msg)

def move_motor(steps, delay=0.005, direction=1):
    _ensure_gpio_ready()
    seq = SEQ if direction == 1 else SEQ[::-1]
    for _ in range(steps):
        for pattern in seq:
            GPIO.output(IN1, pattern[0])
            GPIO.output(IN2, pattern[1])
            GPIO.output(IN3, pattern[2])
            GPIO.output(IN4, pattern[3])
            time.sleep(delay)

def publish_steps():
    if step_pub is not None:
        step_pub.publish(Int32(data=moved_steps))

# === GO ===
def go_mode(should_stop=None):
    """전방 이동: 거리 < 5cm면 정지"""
    global moved_steps
    rospy.loginfo("GO: Moving forward until distance < 5cm")
    send_to_arduino("GO")
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450*8))  #newest added
    step = 1
    miss = 0
    try:
        while not rospy.is_shutdown():
            try:
                dist_cm = listen_from_arduino()
                if dist_cm is None:
                    miss += 1
                    if miss >= 100:
                        rospy.logwarn("no distance received")
                        break
                    continue
                miss = 0
            except Exception as e:
                rospy.logwarn(f"listen_from_arduino() failed : {e}")
                break

            if dist_cm < DIST_THRESHOLD_GO:
                rospy.loginfo("Distance < 5cm -> STOP")
                break

            move_motor(step, delay=delay, direction=1)
            moved_steps += step
            publish_steps()
    finally:
        send_to_arduino("STOP")
        cleanup_gpio()

# === ABNORMAL ===
def abnormal_mode():
    """후진: 거리 > 10cm면 정지 → 시퀀스 전송"""
    global moved_steps
    rospy.loginfo("ABNORMAL: Moving backward until distance > 10cm")
    send_to_arduino("ABNORMAL")
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))
    step = 1
    miss = 0
    try:
        while not rospy.is_shutdown():
            try:
                d = listen_from_arduino()
                if d is None:
                    miss += 1
                    if miss >= 100:
                        rospy.logwarn("no distance received")
                        break
                    continue
                miss = 0
            except Exception as e:
                rospy.logwarn(f"listen_from_arduino() failed : {e}")
                break

            if d > DIST_THRESHOLD_ABNORMAL:
                rospy.loginfo("Distance > 10cm -> STOP")
                break

            move_motor(step, delay=delay, direction=-1)
            moved_steps -= step
            publish_steps()
    finally:
        send_to_arduino("STOP")
        cleanup_gpio()

    # 후속 동작 시리얼 명령(필요 시 유지)
    send_to_arduino("1,F,2.0,5")
    send_to_arduino("2,F,30.0,90")
    time.sleep(5)

# === NORMAL ===
def normal_mode():
    """원위치 복귀: 이동한 스텝만큼 되감기"""
    global moved_steps
    rospy.loginfo(f"NORMAL: Returning {moved_steps} steps forward")
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))
    try:
        if moved_steps > 0:
            move_motor(moved_steps, delay=delay, direction=-1)
        moved_steps = 0
        rospy.loginfo("Return complete.")
        publish_steps()
    finally:
        cleanup_gpio()

# === QUIT ===
def quit_mode():
    """종료: 모든 모터 off + GPIO 해제"""
    rospy.loginfo("QUIT: Turning off all motors")
    send_to_arduino("QUIT")
    cleanup_gpio()

# === (옵션) 엔트리포인트: 노드 초기화/자원 정리 ===
def init_node(node_name="motor_controller"):
    global arduino, step_pub
    rospy.init_node(node_name, anonymous=False)
    step_pub = rospy.Publisher("/steps", Int32, queue_size=10)
    arduino = ArduinoLink(port="/dev/ttyACM0", baud=9600, timeout=1)

def shutdown_node():
    global arduino
    try:
        cleanup_gpio()
    finally:
        if arduino:
            arduino.close()
            arduino = None
