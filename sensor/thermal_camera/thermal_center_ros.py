#!/home/tunnel/jetson_project/yolov_env/bin/python

# === ROS imports for node, message types ===
import rospy                                      # <-- Added for ROS node control
from std_msgs.msg import Int32, Float32          # <-- Added for publishing move commands and positions

import numpy as np
import time
import board
import busio
import adafruit_mlx90640
import cv2

# Initialize MLX90640 sensor (same as original)
i2c = busio.I2C(board.SCL, board.SDA, frequency=1200000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_16_HZ

SHAPE = (24, 32)
frame = np.zeros(24 * 32, dtype=np.float32)

def temp_to_u8(arr, vmin=22.0, vmax=36.0):
    clipped = np.clip(arr, vmin, vmax)
    return ((clipped - vmin) * (255.0 / (vmax - vmin))).astype(np.uint8).reshape(SHAPE)

def thermal_node():
    # Initialize ROS node (replaces script start)
    rospy.init_node('thermal_vision_node')                   # <-- NEW: initialize ROS node

    # Publishers for movement command and optional rightmost_x position
    move_pub = rospy.Publisher('/thermal_move_cmd', Int32, queue_size=1)  # <-- NEW: publish movement commands
    x_pub = rospy.Publisher('/rightmost_x', Float32, queue_size=1)        # <-- NEW: publish rightmost hotspot x-position

    rospy.loginfo("Starting thermal vision node...")
    time.sleep(4)  # wait sensor stabilize

    # Set ROS loop rate (replaces while True with controlled rate)
    rate = rospy.Rate(10)  # 10 Hz                                  # <-- NEW: ROS loop rate

    while not rospy.is_shutdown():                                  # <-- NEW: loop until ROS shuts down
        try:
            mlx.getFrame(frame)                                     # same as original: get thermal frame
        except Exception as e:
            rospy.logwarn(f"MLX90640 frame read failed: {e}")      # <-- NEW: log failure instead of crashing
            rate.sleep()
            continue

        img_f32 = frame.reshape(SHAPE)

        # Gradient magnitude calculation (same as original)
        gy, gx = np.gradient(img_f32)
        grad_mag = np.sqrt(gx**2 + gy**2)

        threshold = 1.5
        mask = (grad_mag > threshold).astype(np.uint8)

        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)

        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

        centers = []
        for i in range(1, n_labels):  # skip background
            area = stats[i, cv2.CC_STAT_AREA]
            if area < 40:
                continue
            cx, cy = centroids[i]
            centers.append((cx, cy))

        if centers:
            # Find rightmost center (same logic)
            rightmost = max(centers, key=lambda p: p[0])
            rightmost_x = rightmost[0] * (640 / SHAPE[1])  # scale x to 640 width (same as original scaling)

            move_x = 320 - rightmost_x  # target center is pixel 320

            # Convert move_x to discrete commands instead of printing
            if move_x > 0:
                move_cmd = 1
            elif move_x < 0:
                move_cmd = -1
            else:
                move_cmd = 0

            move_pub.publish(move_cmd)                     # <-- NEW: publish movement command instead of print
            x_pub.publish(rightmost_x)                     # <-- NEW: publish rightmost x coordinate optionally

            rospy.loginfo(f"Move command: {move_cmd}, Rightmost X: {rightmost_x:.2f}")  # <-- NEW: ROS logging info
        else:
            # No centers found: publish stop command (was not handled before)
            move_pub.publish(0)                           # <-- NEW: publish 0 when no hotspot detected
            rospy.loginfo("No hot spot detected, publishing stop command.")  # <-- NEW: log no hotspot

        rate.sleep()                                      # <-- NEW: control loop frequency with ROS rate

if __name__ == '__main__':
    try:
        thermal_node()
    except rospy.ROSInterruptException:
        pass

