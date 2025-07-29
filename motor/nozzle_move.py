import serial
import time
import matplotlib.pyplot as plt
import numpy as np

ser = serial.Serial('COM11', 9600)
time.sleep(2)

CM_PER_STEP = 1 / 450   #cm당 스텝수 여기를 조정하면 된다!
DEG_PER_STEP = 360 / 2048
MIN_DELAY_MS = 2.0

def send_command(motor_id, direction, speed, distance):     #모터id, 방향, 속도, 거리를 출력/송신
    cmd = f"{motor_id},{direction},{speed:.2f},{distance:.2f}\n"
    ser.write(cmd.encode())
    print(f"[전송] ID:{motor_id}, DIR:{direction}, SPEED:{speed:.2f}, DIST:{distance:.2f}")
    time.sleep(0.05)

def move(path, r_start, theta_start, linear_speed=0, duration=0, visualize=False):  #목적지, 초기값, cm/s, sec, 그림
    if isinstance(path, tuple):
        path = [path]

    r_history = []
    theta_history = []

    curr_r, curr_theta = r_start, theta_start

    for r, theta in path:
        dr = r - curr_r
        dtheta = theta - curr_theta

        if duration > 0:
            move_time = duration
        else:
            t_r = abs(dr) / linear_speed if dr != 0 else 0
            t_theta = abs(dtheta) / (linear_speed * 3) if dtheta != 0 else 0
            move_time = max(t_r, t_theta, 0.1)

        speed_r = dr / move_time if dr != 0 else 0
        speed_theta = dtheta / move_time if dtheta != 0 else 0

        dir_r = 'F' if dr >= 0 else 'B'
        dir_theta = 'F' if dtheta >= 0 else 'B'

        send_command(1, dir_r, abs(speed_r), abs(dr))                #send_command
        send_command(2, dir_theta, abs(speed_theta), abs(dtheta))

        if linear_speed > 0 and visualize:
            dt = 0.05
            t = 0.0
            while t <= move_time:
                interp_r = curr_r + speed_r * t
                interp_theta = curr_theta + speed_theta * t

                if t > 0:
                    r_history.append(interp_r)
                    theta_history.append(interp_theta)

                t += dt

        time.sleep(move_time + 0.5) #다음 신호를 미리 받는다 하면 0.5-> 1로 바꾸든 증가하면 됨!
        curr_r, curr_theta = r, theta

    if linear_speed > 0 and visualize:
        theta_rad = np.deg2rad(theta_history)
        plt.figure(figsize=(6,6))
        ax = plt.subplot(111, polar=True)
        ax.plot(theta_rad, r_history, marker='o', markersize=2)

        # === 이 두 줄을 추가하면 Y축 기준 + 시계방향 됨 ===
        ax.set_theta_zero_location('N')   # 0도 = North = +Y축
        ax.set_theta_direction(-1)        # 시계방향으로 각도 증가

        ax.set_title('Actual Movement Path (Polar Plot)')
        ax.set_rlabel_position(0)
        ax.grid(True)
        plt.show()

    return curr_r, curr_theta

def close_serial():
    ser.close()
