#!/home/tunnel/jetson_project/yolov_env/bin/python
import torch, re
if not isinstance(torch.__version__, str):
    torch.__version__ = re.sub(r'\+.*', '', str(torch.__version__))
import cv2
import numpy as np
import time
from ultralytics import YOLO
from skimage.morphology import skeletonize, remove_small_objects
from coord_change_node import coord_change

# ROS: import ROS-related modules (새로 추가된 부분)
import rospy
from std_msgs.msg import Float32MultiArray

def main():
    # ROS: initialise ROS node (새로 추가된 부분)
    rospy.init_node('tile_outline_publisher')

    # ROS: publisher for flattened (r,θ) list
    #      **topic name 변경 → /tile_center**  (수정된 부분)
    pub = rospy.Publisher('/tile_center',
                          Float32MultiArray, queue_size=1, latch=True)

    # Load YOLO model
    model_path = "/home/tunnel/Desktop/riot/best.pt"
    model = YOLO(model_path)

    # Open camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open camera.")
        return

    # ROS: loop will exit cleanly if ROS master shuts down (새로 추가된 부분)
    while not rospy.is_shutdown():
        loop_start = time.time()

        # Read frame
        t0 = time.time()
        ret, frame = cap.read()
        t1 = time.time()
        capture_time = t1 - t0

        if not ret:
            break

        print("Frame checksum:", np.sum(frame))
        h, w = frame.shape[:2]
        vis = frame.copy()
        overlay = np.zeros_like(frame)
        center = (w / 2, h / 2)  # Image center

        # Run YOLO inference
        t0 = time.time()
        results = model(frame, conf=0.7, device="cuda:0")[0]
        t1 = time.time()
        inference_time = t1 - t0

        # Post-processing
        t0 = time.time()
        found_tile = False  # Flag. if tile is found -> Flag = True, stop loop
        changed_list = []   # store (r,θ) pairs
        if results.masks is not None: # if there is a tile and mask
            for mask_t, cls in zip(results.masks.data, results.boxes.cls):
                if results.names[int(cls.item())] != "tile": # if not tile -> skip
                    continue

                # mask pre process
                mask = mask_t.cpu().numpy().astype(np.uint8) * 255
                area = np.sum(mask > 0)
                # skip noise
                if area < 300 or area > w * h * 0.8:
                    continue

                # if mask is cutted -> ignore
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                mask = cv2.dilate(mask, kernel, iterations=1)

                # get outline, if area is too small -> ignore
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for cnt in cnts:
                    if cv2.contourArea(cnt) < 300:
                        continue

                    # check if area include center
                    if cv2.pointPolygonTest(cnt, center, False) >= 0:
                        contour_coords = cnt.reshape(-1,2) # change every contour(outline) into x,y coordinate as numpy array
                        #contour_x = contour_coords[:, 0] >> this is total array of x coordinate
                        #contour_y = contour_coords[:, 1]

                        changed_list = [ ]
                        for x,y in contour_coords:
                            r, theta = coord_change(float(x), float(y), 320, 240)
                            changed_list.append( (r,theta) )

                        # visualize outline
                        cv2.drawContours(vis, [cnt], -1, (0, 0, 255), 2)
                        overlay[mask == 255] = (0, 255, 0)

                        #tile found, end loop
                        found_tile = True
                        break  

                if found_tile:
                    print(changed_list)
                    # ROS: publish flattened list as Float32MultiArray
                    flat = [val for pair in changed_list for val in pair]
                    pub.publish(Float32MultiArray(data=flat))
                    break

        # Blend overlay and draw center point
        vis = cv2.addWeighted(vis, 1.0, overlay, 0.3, 0)
        cv2.circle(vis, (int(center[0]), int(center[1])), 5, (255, 0, 255), -1)

        t1 = time.time()
        post_time = t1 - t0

        # Show the output
        cv2.imshow("Tile Detection", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        loop_end = time.time()
        loop_time = loop_end - loop_start

        print(f" Capture: {capture_time*1000:.2f}ms | Inference: {inference_time*1000:.2f}ms | Post: {post_time*1000:.2f}ms | Total: {loop_time*1000:.2f}ms\n")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()