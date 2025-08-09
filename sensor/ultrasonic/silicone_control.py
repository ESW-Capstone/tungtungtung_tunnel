import serial
import time
import threading
import RPi.GPIO as GPIO

# === GPIO Setup for Stepper Motor ===
IN1, IN2, IN3, IN4 = 17, 18, 27, 22  # BCM
GPIO.setmode(GPIO.BCM)
for pin in [IN1, IN2, IN3, IN4]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

# === Step Sequence for 28BYJ-48 ===
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

# === Constants ===
CM_PER_STEP = 1.0 / 450.0
DEFAULT_SPEED_CM_S = 2.0
DIST_THRESHOLD_GO = 5.0
DIST_THRESHOLD_ABNORMAL = 10.0

# === Serial to Arduino ===
arduino = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

# === Shared State ===
distance_cm = 999.0
moved_steps = 0
lock = threading.Lock()

# === Stepper Motor Functions ===
def move_motor(steps, delay=0.002, direction=1):
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

# === Serial Receive Thread ===
def listen_from_arduino():
    global distance_cm
    while True:
        if arduino.in_waiting:
            line = arduino.readline().decode().strip()
            if line.startswith("DIST,"):
                try:
                    dist = float(line.split(',')[1])
                    with lock:
                        distance_cm = dist
                    print(f"[DIST] {dist:.2f} cm")
                except:
                    pass
        time.sleep(0.05)

# === Arduino Command Sender ===
def send_to_arduino(msg):
    arduino.write((msg + '\n').encode())
    print(f"[TX ??Arduino] {msg}")

# === GO Mode ===
def go_mode():
    global moved_steps
    print("GO: Moving forward until distance < 5cm")
    send_to_arduino("GO")
    step = 1
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))

    while True:
        with lock:
            if distance_cm < DIST_THRESHOLD_GO:
                print("??Distance < 5cm ??STOP")
                break
        move_motor(step, delay=delay, direction=1)
        moved_steps += step

    send_to_arduino("STOP")
    cleanup()
    time.sleep(0.5)
    send_to_arduino("HIT")

# === ABNORMAL Mode ===
def abnormal_mode():
    global moved_steps
    print("??ABNORMAL: Moving backward until distance > 10cm")
    send_to_arduino("ABNORMAL")
    step = 1
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))

    while True:
        with lock:
            if distance_cm > DIST_THRESHOLD_ABNORMAL:
                print("??Distance > 10cm ??STOP")
                break
        move_motor(step, delay=delay, direction=-1)
        moved_steps -= step
    
    send_to_arduino("STOP")
    cleanup()
    print("??Sending r and theta motor commands")
    send_to_arduino("1,F,2.0,5")
    send_to_arduino("2,F,30.0,90")
    time.sleep(5)

# === NORMAL Mode ===
def normal_mode():
    global moved_steps
    print(f"??NORMAL: Returning {moved_steps} steps forward")
    delay = max(0.002, 1.0 / (DEFAULT_SPEED_CM_S * 450))
    move_motor(moved_steps, delay=delay, direction=-1)
    moved_steps = 0
    print("??Return complete.")

# === QUIT ===
def quit_mode():
    print("??QUIT: Turning off all motors")
    send_to_arduino("QUIT")
    cleanup()
    GPIO.cleanup()

# === User Input ===
def handle_user_input():
    print("Commands: go / abnormal / normal / quit")
    while True:
        cmd = input("\n>> Command: ").strip().lower()
        if cmd == "go":
            go_mode()
        elif cmd == "abnormal":
            abnormal_mode()
        elif cmd == "normal":
            normal_mode()
        elif cmd == "quit":
            quit_mode()
            break
        else:
            print("Invalid command.")

# === Main Execution ===
try:
    threading.Thread(target=listen_from_arduino, daemon=True).start()
    handle_user_input()
except KeyboardInterrupt:
    quit_mode()