#!/home/tunnel/jetson_project/yolov_env/bin/python
import time
import threading
import sys
import serial
import rospy
from smbus2 import SMBus, i2c_msg
from std_msgs.msg import String, Int32

from camera_path import capture_path_rtheta_validated
# 시리얼 관련은 arm_control만 사용 (포트 오픈/닫기 포함)
from arm_control import (send_to_arduino, abnormal_mode, quit_mode, shutdown_node)

# ============ ROS Param Helper ============
def P(name, default):
    return rospy.get_param("~" + name, default)

# ============ I2C Helper Class ============

class ArduinoI2C:
    """단일 I2C 대상 주소로 텍스트 명령 전송"""
    def __init__(self, bus: SMBus, addr: int):
        self.bus = bus
        self.addr = addr

    def send_text(self, text: str):
        try:
            data = text.encode('ascii')[:31]  # Wire 32-byte limit
            msg = i2c_msg.write(self.addr, data)
            self.bus.i2c_rdwr(msg)
            rospy.loginfo(f"[I2C->0x{self.addr:02X}] {text}")
        except Exception as e:
            rospy.logwarn(f"[I2C->0x{self.addr:02X}] send fail: {e}")

# ============ Emergency Stop ============
_emergency_evt = threading.Event()

def emergency_stop(reason="keyboard"):
    """Hard stop everything we control."""
    try:
        _emergency_evt.set()
        rospy.logwarn(f"[EMERGENCY] stop triggered ({reason})")

        # I2C 쪽: 존재하지 않는 pillar_stop() 대신 텍스트 명령으로 안전정지 지시(있다면)
        try:
            if 'A2' in globals() and A2:
                A2.send_text("PILLAR_STOP")
        except Exception as e:
            rospy.logwarn(f"[EMERGENCY] I2C stop fail: {e}")

        # UART 쪽: arm_control 함수만 사용
        try:
            quit_mode()
        except Exception as e:
            rospy.logwarn(f"[EMERGENCY] quit_mode failed: {e}")

        try:
            send_to_arduino("STOP", ensure_open=False)  # 새로 열지 않음
        except Exception as e:
            rospy.logwarn(f"[EMERGENCY] UART notify failed: {e}")

    except Exception as e:
        rospy.logwarn(f"[EMERGENCY] unexpected error: {e}")

def keyboard_listener_thread():
    """stdin에서 'st' 입력 시 비상정지"""
    rospy.loginfo("[KEY] Listener started (type 'st' + Enter to emergency-stop)")
    while not rospy.is_shutdown():
        try:
            line = sys.stdin.readline()
            if not line:
                time.sleep(0.05); continue
            if line.strip().lower() == "st":
                emergency_stop("keyboard:st")
        except Exception as e:
            rospy.logwarn(f"[KEY] error: {e}")
            time.sleep(0.1)

# ============ Mode2 Node ============
class Mode2Node:
    def __init__(self, a2: ArduinoI2C):
        self.a2 = a2
        # 파라미터
        self.linear_speed = P("linear_speed_cm_s", 3.0)
        self.theta_scale  = P("theta_speed_scale", 3.0)
        self.segment_pad  = P("segment_settle_sec", 0.5)
        self.shoot_ms     = P("shoot_ms", 800)
        self.model_path   = P("model_path", "/home/tunnel/Desktop/riot/best.pt")
        self.cam_index    = P("camera_index", 0)
        self.conf_thr     = P("conf", 0.7)
        self.step_px      = P("step_px", 5.0)
        self.cx           = P("cx", 0.0)
        self.cy           = P("cy", 0.0)
        self.px2cm_x      = P("px2cm_x", 1.0)
        self.px2cm_y      = P("px2cm_y", 1.0)
        self.device       = P("device", "cuda:0")
        self.sort_from_90 = P("sort_from_90", True)

        self.initial_r     = P("initial_r", 0.0)
        self.initial_theta = P("initial_theta", -90.0)
        self.pre_r         = P("pre_r", 0.0)
        self.pre_theta     = P("pre_theta", 90.0)
        self.post_r        = P("post_r", 0.0)
        self.post_theta    = P("post_theta", -90.0)

        self.pub_done = rospy.Publisher("/mode_result", String, queue_size=1, latch=True)

        # 스텝→시간 단순 추정용
        self.cm_per_step = P("cm_per_step", 1.0/450.0)

        self._ser_lock = threading.Lock()

    # ===== UART helpers =====
    def send_text(self, text: str, sleep_after=0.02):
        with self._ser_lock:
            send_to_arduino(text, ensure_open=True)  # 필요 시 자동 오픈
        if sleep_after > 0:
            time.sleep(sleep_after)

    def send_move(self, motor_id: int, direction: str, speed: float, distance: float, sleep_after=0.05):
        cmd = f"{motor_id},{direction},{speed:.2f},{distance:.2f}"
        self.send_text(cmd, sleep_after=sleep_after)

    def _send_delta_move(self, dr, dth):
        if abs(dr) > 1e-6:
            self.send_move(1, 'F' if dr >= 0 else 'B', self.linear_speed, abs(dr))
        if abs(dth) > 1e-6:
            self.send_move(2, 'F' if dth >= 0 else 'B', self.linear_speed * self.theta_scale, abs(dth))

    def _estimate_move_time(self, abs_dr, abs_dth):
        t_r  = (abs_dr  / max(self.linear_speed, 1e-6)) if abs_dr  > 0 else 0.0
        t_th = (abs_dth / max(self.linear_speed * self.theta_scale, 1e-6)) if abs_dth > 0 else 0.0
        return max(t_r, t_th, 0.1)

    def _publish_results_and_cleanup(self):
        try:
            self.send_text("STOP", sleep_after=0.02)
            if self.a2: self.a2.send_text("shoot_stop")
            self.send_text("QUIT", sleep_after=0.02)
            self.pub_done.publish(String(data="done"))
        except Exception as e:
            rospy.logwarn(f"[mode2] cleanup error: {e}")

    # ===== Main sequence =====
    def run(self):
        try:
            # 1) 비정상 시퀀스(후진) 실행
            rospy.loginfo("[mode2] abnormal_mode()")
            abnormal_mode()

            # 2) 프리 포지션 이동
            dr0  = self.pre_r - self.initial_r
            dth0 = self.pre_theta - self.initial_theta
            rospy.loginfo(f"[mode2] pre-position to ({self.pre_r:.2f}, {self.pre_theta:.2f}) (Δr={dr0:.2f}, Δθ={dth0:.2f})")
            self._send_delta_move(dr0, dth0)
            time.sleep(self._estimate_move_time(abs(dr0), abs(dth0)) + self.segment_pad)

            # 3) 경로 캡처
            rospy.loginfo("[mode2] capturing path (validated one-shot)...")
            path = capture_path_rtheta_validated(
                model_path=self.model_path,
                cam_index=self.cam_index,
                conf_thr=self.conf_thr,
                step_px=self.step_px,
                cx=(None if self.cx == 0.0 else self.cx),
                cy=(None if self.cy == 0.0 else self.cy),
                px2cm_x=self.px2cm_x,
                px2cm_y=self.px2cm_y,
                do_sort_from_90=self.sort_from_90,
                device=self.device,
                max_attempts=P("max_attempts", 10),
                timeout_s=P("capture_timeout_sec", 8.0),
                min_points=P("min_points", 16),
                debug_view=P("debug_view", False)
            )

            if not path:
                rospy.logwarn("[mode2] empty path. cleanup.")
                self._publish_results_and_cleanup()
                return

            # 4) 포인트별 이동 + 타격
            prev_r, prev_th = self.pre_r, self.pre_theta
            for idx, (r, th) in enumerate(path, start=1):
                dr  = r  - prev_r
                dth = th - prev_th
                if self.a2: self.a2.send_text("shoot")
                self._send_delta_move(dr, dth)
                move_time = self._estimate_move_time(abs(dr), abs(dth))
                time.sleep(move_time + self.segment_pad)

                if self.a2:
                    self.a2.send_text("shoot_rev")
                    time.sleep(1)
                    self.a2.send_text("shoot_stop")
                rospy.loginfo(f"[mode2] segent {idx}/{len(path)} done")
                prev_r, prev_th = r, th

            # 5) 포스트 포지션
            drp  = self.post_r - prev_r
            dthp = self.post_theta - prev_th
            rospy.loginfo(f"[mode2] post-position to ({self.post_r:.2f}, {self.post_theta:.2f}) (Δr={drp:.2f}, Δθ={dthp:.2f})")
            self._send_delta_move(drp, dthp)
            time.sleep(self._estimate_move_time(abs(drp), abs(dthp)) + self.segment_pad)

            # 6) 정리/발행
            self._publish_results_and_cleanup()
            rospy.loginfo("[mode2] finished. Ctrl+C to exit.")

            r = rospy.Rate(5)
            while not rospy.is_shutdown() and not _emergency_evt.is_set():
                r.sleep()

        finally:
            # arm_control 쪽 자원정리는 shutdown_node()에서
            pass

# ============ main ============
def main():
    global A2

    rospy.init_node("mode2_node")

    # 키보드 비상정지 스레드
    threading.Thread(target=keyboard_listener_thread, daemon=True).start()

    # I2C 준비
    bus_no   = P("i2c_bus_no", 0)
    addr_a2  = P("arduino2_addr", 0x18)  # pillar/slide

    bus = None
    try:
        bus = SMBus(bus_no)
        A2 = ArduinoI2C(bus, addr_a2)
        rospy.loginfo(f"[I2C] bus {bus_no} ready, A2=0x{addr_a2:02X}")
    except Exception as e:
        rospy.logwarn(f"[I2C] open fail: {e}")

    node = Mode2Node(A2)

    try:
        node.run()
    finally:
        # I2C close
        try:
            if bus is not None:
                bus.close()
        except:
            pass
        # arm_control 정리(시리얼/GPIO 등)
        shutdown_node()
        rospy.loginfo("mode2_node exit")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        emergency_stop("KeyboardInterrupt")
