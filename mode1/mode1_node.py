#!/home/tunnel/jetson_project/yolov_env/bin/python

# after car move to x coord, servo motor angle set to 0
# x move -> pillar up, y move -> stop -> hit ( abnormal  normal )  

import rospy
import serial
import threading
import time
from std_msgs.msg import String, Float32, Int32

from arm_control import 
(
    go_mode, abnormal_mode, normal_mode, quit_mode, distance_callback
)

from angle_calculate import listen_from_arduino, close_arduino

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

# -------------------
# Parameters
# -------------------
def param(name, default):
    return rospy.get_param("~" + name, default)

UART1_PORT  = None      # Arduino1 (UART)
UART1_BAUD  = 9600

I2C_BUS_NO  = 1         # Common I2C bus
I2C_ADDR2   = 0x10      # Arduino2 address (e.g., pillar/slide)
I2C_ADDR3   = 0x11      # Arduino3 address (e.g., W/S drive)

CENTER_TOPIC = "/center"         # Float32: 0=center
SOUND_TOPIC  = "/sound_result"   # String: "normal"/"abnormal"

CENTER_DEADBAND = 3.0
CENTER_COOLDOWN = 0.15
CHECK_WAIT_SEC  = 8.0

# -------------------
# Global state
# -------------------
manual_lock = threading.Lock()
manual_command = None  # 'go' | 'quit' | 'normal' | 'abnormal' | 'pillar_stop'

uart1_ser = None   # Arduino1(UART)
i2c_bus   = None   # Common I2C

last_center_tx = 0.0
aligned = False

sound_result = None
sound_lock = threading.Lock()

# -------------------
# Communication utilities
# -------------------
def open_uart(port, baud, name):
    if not port:
        return None
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        rospy.loginfo(f"[{name}] open {port}@{baud}")
        return ser
    except Exception as e:
        rospy.logwarn(f"[{name}] open fail: {e}")
        return None

def i2c_send_text(addr, text):
    """Arduino Wire (assuming 32-byte limit)"""
    if i2c_bus is None:
        rospy.logwarn("[I2C] bus not available")
        return
    try:
        data = bytearray(text.encode('ascii'))[:31]  # cmd=0x00, payload max 31
        payload = [c for c in data]
        i2c_bus.write_i2c_block_data(addr, 0x00, payload)
        rospy.loginfo(f"[I2C->0x{addr:02X}] {text}")
    except Exception as e:
        rospy.logwarn(f"[I2C->0x{addr:02X}] send fail: {e}")

# -------------------
# Callback: center ¡æ Arduino3(I2C) W/S
# -------------------
def center_callback(msg: Float32): # this function : move car. in this function : have_x -> have to move car
    global last_center_tx, aligned
    c = msg.data
    now = time.time()
    if abs(c) <= CENTER_DEADBAND:
        aligned = True
        return
    aligned = False
    if now - last_center_tx < CENTER_COOLDOWN:
        return
    # Convention: c>0 ¡æ W, c<0 ¡æ S (reverse if needed)
    cmd = "w" if c > 0 else "s" # w : move forward, s : move backward, q : stop
    i2c_send_text(I2C_ADDR3, cmd) # this send w,s,q to arduino3 by i2c. 
    rospy.loginfo_throttle(1.0, f"[A3 I2C] {cmd} (center={c:.2f})")
    last_center_tx = now

# -------------------
# Callback: sound_result
# -------------------
def sound_result_callback(msg: String):
    global sound_result
    with sound_lock:
        sound_result = msg.data.strip().lower()
        rospy.loginfo(f"[sound_result] {sound_result}")

def wait_sound_result(timeout=CHECK_WAIT_SEC):
    global sound_result
    t0 = time.time()
    while time.time() - t0 < timeout and not rospy.is_shutdown():
        with sound_lock:
            if sound_result in ("normal", "abnormal"):
                return sound_result
        time.sleep(0.1)
    return None

# -------------------
# Manual input thread
# -------------------
def manual_input_thread():
    global manual_command
    help_txt = (
        "\n[Manual] Command:\n"
        "  go            : start go_mode\n"
        "  normal        : manually set result to normal after CHECK\n"
        "  abnormal      : manually set result to abnormal after CHECK\n"
        "  pillar_stop   : Arduino2(I2C) pillar stop\n"
        "  quit          : full quit sequence\n"
    )
    print(help_txt)
    while not rospy.is_shutdown():
        try:
            cmd = input(">> ").strip().lower()
        except EOFError:
            break
        with manual_lock:
            manual_command = cmd

# -------------------
# Arduino1(UART) receive thread (STOP / HIT_READY / CHECK)
# -------------------
def uart1_listener_thread(pub_mode_result):
    """
    Lines from Arduino1:
      STOP      ¡æ slide finished
      HIT_READY ¡æ ready to record
      CHECK     ¡æ enter sound analysis stage ¡æ wait for sound_result ¡æ if none, wait for manual input
    """
    if uart1_ser is None:
        rospy.logwarn("[UART1] listener disabled (no port)")
        return
    while not rospy.is_shutdown():
        try:
            line = uart1_ser.readline().decode(errors='ignore').strip()
            if not line:
                time.sleep(0.05)
                continue
            u = line.upper()
            rospy.loginfo(f"[UART1 RX] {u}")

            if u == "STOP":
                rospy.loginfo(" ¡æ [ACTION] Slide movement complete")

            elif u == "HIT_READY":
                rospy.loginfo(" ¡æ [ACTION] Recording ready")

            elif u == "CHECK":
                rospy.loginfo(" ¡æ [ACTION] Start sound analysis (waiting for result)")
                res = wait_sound_result()
                if res not in ("normal", "abnormal"):
                    rospy.loginfo(" ¡æ [ACTION] No topic result. Waiting for manual input (normal/abnormal)")
                    # Manual input polling
                    while not rospy.is_shutdown():
                        with manual_lock:
                            cmd = manual_command
                            if cmd in ("normal", "abnormal"):
                                res = cmd
                                manual_command = None
                                break
                        time.sleep(0.1)
                if res not in ("normal", "abnormal"):
                    rospy.logwarn(" ¡æ [ACTION] No result. Default abnormal")
                    res = "abnormal"

                pub_mode_result.publish(res)
                rospy.loginfo(f"[mode_result] {res}")

                if res == "normal":
                    normal_mode()
                else:
                    abnormal_mode()

        except Exception as e:
            rospy.logwarn(f"[UART1 RX] error: {e}")
            time.sleep(0.1)


def x_control(topic='/have_to_move_x', i2c_addr=None, tx_cooldown=0.20, stop_hold=0.6, rate_hz=20):
    if i2c_addr is None:
        i2c_addr = I2C_ADDR3
    prev_cmd = None
    last_tx  = 0.0
    stop_since = None
    done = False
    def cb(msg: Int32):
        nonlocal prev_cmd, last_tx, stop_since, done
        v = int(msg.data)
        cmd = 'w' if v > 0 else ('s' if v < 0 else 'q')
        now = time.time()
        if cmd != prev_cmd or (now - last_tx) >= tx_cooldown:
            i2c_send_text(i2c_addr, cmd)
            prev_cmd = cmd
            last_tx  = now
        if cmd == 'q':
            if stop_since is None:
                stop_since = now
            elif (now - stop_since) >= stop_hold:
                done = True
        else:
            stop_since = None
    sub = rospy.Subscriber(topic, Int32, cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown() and not done:
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        i2c_send_text(i2c_addr, 'q')
    return True

def y_control(topic='/have_to_move_y', i2c_addr=None, tx_cooldown=0.20, stop_hold=0.6, rate_hz=20):
    if i2c_addr is None:
        i2c_addr = I2C_ADDR2
    prev_cmd = None
    last_tx  = 0.0
    stop_since = None
    done = False
    def cb(msg: Int32):
        nonlocal prev_cmd, last_tx, stop_since, done
        v = int(msg.data)
        if v>0:
            cmd = 'UP'
        else:
            cmd = 'PILLAR_STOP'
        now = time.time()
        if cmd != prev_cmd or (now - last_tx) >= tx_cooldown:
            i2c_send_text(i2c_addr, cmd)
            prev_cmd = cmd
            last_tx  = now
        if cmd == 'PILLAR_STOP':
            if stop_since is None:
                stop_since = now
            elif (now - stop_since) >= stop_hold:
                done = True
        else:
            stop_since = None
    sub = rospy.Subscriber(topic, Int32, cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown() and not done:
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        i2c_send_text(i2c_addr, 'PILLAR_STOP')
    return True

def run_mode1_sequence():
    ok_x = x_control()
    if not ok_x:
        rospy.logwarn("[MODE1] X-phase ended unexpectedly; continuing to Y-phase")
    ok_y = y_control()
    if not ok_y:
        rospy.logwarn("[MODE1] Y-phase ended unexpectedly")
    rospy.loginfo("[MODE1] sequence complete")
    return ok_x and ok_y


# -------------------
# Main
# -------------------
def main():
    global uart1_ser, i2c_bus
    rospy.init_node("mode1_node")

    # Load parameters
    global UART1_PORT, UART1_BAUD, I2C_BUS_NO, I2C_ADDR2, I2C_ADDR3
    UART1_PORT = param("uart1_port",  "/dev/ttyTHS1")  # Arduino1
    UART1_BAUD = param("uart1_baud",  9600)
    I2C_BUS_NO = param("i2c_bus_no", 1)
    I2C_ADDR2  = param("arduino2_addr", 0x10)
    I2C_ADDR3  = param("arduino3_addr", 0x04)

    # Initialize communication
    uart1_ser = open_uart(UART1_PORT, UART1_BAUD, "UART1")

    if SMBus is not None:
        try:
            i2c_bus = SMBus(I2C_BUS_NO)
            rospy.loginfo(f"[I2C] bus {I2C_BUS_NO} ready, A2=0x{I2C_ADDR2:02X}, A3=0x{I2C_ADDR3:02X}")
        except Exception as e:
            rospy.logwarn(f"[I2C] open fail: {e}")
            i2c_bus = None
    else:
        rospy.logwarn("[I2C] smbus2 not installed; I2C control unavailable")

    ok = run_mode1_sequence()
    rospy.loginfo(f"mode 1 done, ok={ok}") # y end 


    distance = listen_from_arduino()
    go_mode(distance)
    sound_val= sound_test() # 1 : bad , 0 : good
    if sound_val == 1:
        sound_test_answer = "abnormal"
    elif sound_val == 0:
        sound_test_answer = "normal"
    sound_pub = rospy.Publisher("/mode_result", String, queue_size = 1)
    sound_pub.publish(String(data=sound_test_answer))


    rospy.sleep(0.5)


    # 1) Manual input thread
    threading.Thread(target=manual_input_thread, daemon=True).start()

    # 2) Arduino1 listener (handles STOP/HIT_READY/CHECK events)
    threading.Thread(target=uart1_listener_thread, args=(pub_mode_result,), daemon=True).start()

    rospy.loginfo("== mode1_node ready ==")
    rospy.loginfo("Sends W/S to Arduino3(I2C) based on center. After alignment, enter 'go' to execute go_mode.")

    # Main loop: handle go / pillar_stop / quit
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        cmd = None
        with manual_lock:
            if manual_command:
                cmd = manual_command
                manual_command = None

        if cmd == "go":
            rospy.loginfo("[Manual] GO")
            go_mode()

        elif cmd == "pillar_stop":
            i2c_send_text(I2C_ADDR2, "PILLAR_STOP")

        elif cmd == "quit":
            rospy.loginfo("[Manual] QUIT")
            quit_mode()
            break

        rate.sleep()

    rospy.loginfo("mode1_node exit")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass


