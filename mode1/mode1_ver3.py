#!/home/tunnel/jetson_project/yolov_env/bin/python

import rospy
import serial
import threading
import time
import sys
from std_msgs.msg import String, Float32, Int32
from smbus2 import i2c_msg

from arm_control import (go_mode, abnormal_mode, normal_mode, quit_mode, for_step_publish)
from angle_calculate import listen_from_arduino, close_arduino
from sound_data import sound_data

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

def param(name, default):
    return rospy.get_param("~" + name, default)

UART1_PORT = None
UART1_BAUD = 9600

I2C_BUS_NO = 1
I2C_ADDR2  = 0x18  # pillar/slide
I2C_ADDR3  = 0x08 # W/S drive (confirmed)

CENTER_DEADBAND = 3.0
CENTER_COOLDOWN = 0.15

manual_lock = threading.Lock()
manual_command = None

uart1_ser = None
i2c_bus   = None

last_center_tx = 0.0
aligned = False

# === Emergency stop ===
_emergency_evt = threading.Event()

def emergency_stop(reason="keyboard"):
    """Hard stop everything we control."""
    try:
        _emergency_evt.set()
        rospy.logwarn(f"[EMERGENCY] stop triggered ({reason})")

        # I2C hard stop: drive & pillar
        try:
            i2c_send_text(I2C_ADDR3, 'q')               # stop W/S drive
        except Exception as e:
            rospy.logwarn(f"[EMERGENCY] drive stop fail: {e}")
        try:
            i2c_send_text(I2C_ADDR2, 'PILLAR_STOP')     # stop pillar
        except Exception as e:
            rospy.logwarn(f"[EMERGENCY] pillar stop fail: {e}")

        # Optional: higher-level quit if available
        try:
            quit_mode()
        except Exception as e:
            rospy.logwarn(f"[EMERGENCY] quit_mode failed: {e}")

        # Optional: notify over UART (if your firmware watches this)
        try:
            if uart1_ser and uart1_ser.is_open:
                uart1_ser.write(b"STOP\n")
                uart1_ser.flush()
        except Exception as e:
            rospy.logwarn(f"[EMERGENCY] UART notify failed: {e}")

    except Exception as e:
        rospy.logwarn(f"[EMERGENCY] unexpected error: {e}")

def keyboard_listener_thread():
    """Blocks on stdin and triggers emergency_stop when 'st' is typed."""
    rospy.loginfo("[KEY] Listener started (type 'st' + Enter to emergency-stop)")
    while not rospy.is_shutdown():
        try:
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.05)
                continue
            if line.strip().lower() == "st":
                emergency_stop("keyboard:st")
        except Exception as e:
            rospy.logwarn(f"[KEY] error: {e}")
            time.sleep(0.1)

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
    if i2c_bus is None:
        rospy.logwarn("[I2C] bus not available")
        return
    try:
        # NOTE: If your Arduino uses Wire (32-byte limit), keep text <= 31 bytes.
        data = text.encode('ascii')[:31]
        msg = i2c_msg.write(addr, data)
        i2c_bus.i2c_rdwr(msg)
        #rospy.loginfo(f"[I2C->0x{addr:02X}] {text}")
    except Exception as e:
        rospy.logwarn(f"[I2C->0x{addr:02X}] send fail: {e}")

def uart1_listener_thread():
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
            #rospy.loginfo(f"[UART1 RX] {u}")
            if u == "STOP":
                # Tie incoming STOP to actual emergency stop as well
                emergency_stop("uart:STOP")
            elif u == "HIT_READY":
                rospy.loginfo("[ACTION] Recording ready")
        except Exception as e:
            rospy.logwarn(f"[UART1 RX] error: {e}")
            time.sleep(0.1)

def x_control(topic='/have_to_move_x', i2c_addr=None, tx_cooldown=0.20, stop_hold=0.6, rate_hz=20):
    if i2c_addr is None:
        i2c_addr = I2C_ADDR3
    prev_cmd = None
    last_tx  = 0.0
    done = False
    def cb(msg: Int32):
        nonlocal prev_cmd, last_tx, done
        if _emergency_evt.is_set():
            done = True
            return
        v = int(msg.data)
        cmd = 'w' if v > 0 else ('s' if v < 0 else 'q')
        now = time.time()
        if cmd != prev_cmd or (now - last_tx) >= tx_cooldown:
            i2c_send_text(i2c_addr, cmd)
            prev_cmd = cmd
            last_tx  = now
        if cmd == 'q':
            done = True
    sub = rospy.Subscriber(topic, Int32, cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown() and not done and not _emergency_evt.is_set():
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        i2c_send_text(i2c_addr, 'q')
    return not _emergency_evt.is_set()

def y_control(topic='/have_to_move_y', i2c_addr=None, tx_cooldown=0.20, rate_hz=20):
    if i2c_addr is None:
        i2c_addr = I2C_ADDR2
    prev_cmd = None
    last_tx  = 0.0
    done = False
    def cb(msg: Int32):
        nonlocal prev_cmd, last_tx, done
        if _emergency_evt.is_set():
            done = True
            return
        v = int(msg.data)
        cmd = 'UP' if v > 0 else 'PILLAR_STOP'
        now = time.time()
        if cmd != prev_cmd or (now - last_tx) >= tx_cooldown:
            i2c_send_text(i2c_addr, cmd)
            prev_cmd = cmd
            last_tx  = now
        if cmd == 'PILLAR_STOP':
             done = True
    sub = rospy.Subscriber(topic, Int32, cb, queue_size=1)
    r = rospy.Rate(rate_hz)
    try:
        while not rospy.is_shutdown() and not done and not _emergency_evt.is_set():
            r.sleep()
    finally:
        try: sub.unregister()
        except: pass
        i2c_send_text(i2c_addr, 'PILLAR_STOP')
    return not _emergency_evt.is_set()

def run_mode1_sequence():
    ok_x = x_control()
    if not ok_x:
        rospy.logwarn("[MODE1] X-phase ended (possibly emergency); continuing to Y-phase")
    if _emergency_evt.is_set():
        return False
    i2c_send_text(I2C_ADDR3, 'b0')
    ok_y = y_control()
    if not ok_y:
        rospy.logwarn("[MODE1] Y-phase ended (possibly emergency)")
    rospy.loginfo("[MODE1] sequence complete")
    return (ok_x and ok_y and not _emergency_evt.is_set())

def main():
    global uart1_ser, i2c_bus
    rospy.init_node("mode1_node")

    global UART1_PORT, UART1_BAUD, I2C_BUS_NO, I2C_ADDR2, I2C_ADDR3
    UART1_PORT = param("uart1_port",  "/dev/ttyACM0")
    UART1_BAUD = param("uart1_baud",  9600)
    I2C_BUS_NO = param("i2c_bus_no",  0)
    I2C_ADDR2  = param("arduino2_addr", 0x18)
    I2C_ADDR3  = param("arduino3_addr", 0x08)  # stays 0x08

    # Start keyboard listener first so 'st' works anytime.
    threading.Thread(target=keyboard_listener_thread, daemon=True).start()

    uart1_ser = open_uart(UART1_PORT, UART1_BAUD, "UART1")
    threading.Thread(target=uart1_listener_thread, daemon=True).start()

    if SMBus is not None:
        try:
            i2c_bus = SMBus(I2C_BUS_NO)
            rospy.loginfo(f"[I2C] bus {I2C_BUS_NO} ready, A2=0x{I2C_ADDR2:02X}, A3=0x{I2C_ADDR3:02X}")
        except Exception as e:
            rospy.logwarn(f"[I2C] open fail: {e}")
            i2c_bus = None
    else:
        rospy.logwarn("[I2C] smbus2 not installed; I2C control unavailable")

    if not _emergency_evt.is_set():
        ok = run_mode1_sequence()
        rospy.loginfo(f"mode 1 sequence done, ok={ok}")

    if not _emergency_evt.is_set():
        # Distance safety: obtain value before go_mode
        distance = listen_from_arduino()
        if distance is None:
            rospy.logwarn("distance not received; using default=100")
            distance = 100.0
        try:
            go_mode(distance)
        except Exception as e:
            rospy.logwarn(f"go_mode failed: {e}")

    if not _emergency_evt.is_set():
        sound_val = sound_data()  # 1: bad, 0: good, -1/None: invalid
        if sound_val == 1:
            sound_answer = "abnormal"
        elif sound_val == 0:
            sound_answer = "normal"
        else:
            sound_answer = None
            rospy.loginfo("no valid sound result")

        sound_pub = rospy.Publisher("/mode_result", String, queue_size=1)
        if sound_answer is not None:
            sound_pub.publish(String(data=sound_answer))
        else:
            rospy.logwarn("publish skipped (no valid result)")
        rospy.sleep(0.5)
        for_step_publish()

    try:
        # Idle until shutdown or emergency; keeps node alive to catch 'st'
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and not _emergency_evt.is_set():
            r.sleep()
    finally:
        try: close_arduino()
        except: pass
        try:
            if uart1_ser and uart1_ser.is_open:
                uart1_ser.close()
        except: pass
        try:
            if i2c_bus: i2c_bus.close()
        except: pass
        rospy.loginfo("mode1_node exit")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        emergency_stop("KeyboardInterrupt")
