
'''
short explanation

this code find hot area using gradient. maybe it can be controlled by adjusting value of threshold whatever you want. << ask gpt...
after it find hot place, it caculate it's center and center is saved to cx, cy. ( cx, cy is coordinate of "before" resized. px, py is after resized.)
after it find center

1) find rightmost of center comparing x coordinate
2) caculate how much should car move

'''

import time, board, busio
import numpy as np
import adafruit_mlx90640
import datetime as dt
import cv2



# basic settings
i2c = busio.I2C(board.SCL, board.SDA, frequency=1200000)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_16_HZ # set to 16HZ

SHAPE   = (24, 32)                      
frame   = np.zeros(24 * 32, dtype=np.float32)

# pre process
def temp_to_u8(arr, vmin=22.0, vmax=36.0): # cut less than 22, over 36
    clipped = np.clip(arr, vmin, vmax)
    return ((clipped - vmin) * (255.0 / (vmax - vmin))).astype(np.uint8).reshape(SHAPE)

print("code start"); time.sleep(4)
t0 = time.time()

try:
    while True:
       
        # get mlx90640 frame & make gray scale
        mlx.getFrame(frame)
        img_f32 = frame.reshape(SHAPE)
        u8_img = temp_to_u8(img_f32)  

        # apply colormap                      
        vis    = cv2.applyColorMap(u8_img, cv2.COLORMAP_JET)

        #resize
        vis    = cv2.resize(vis, (640, 480), cv2.INTER_CUBIC)


        # from here, ask gpt..
        # === compute gradient magnitude map ===
        gy, gx = np.gradient(img_f32)
        grad_mag = np.sqrt(gx**2 + gy**2)

        # === threshold to binary mask ===
        threshold = 1.5
        mask = (grad_mag > threshold).astype(np.uint8)

        # === apply morphology to clean mask ===
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)

        # === find connected components (blobs) ===
        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

        scale_x = 640 / 32
        scale_y = 480 / 24

        # until here

        centers = [ ] # for storing center coordinate 

        for i in range(1, n_labels):  # skip background (label 0)
            area = stats[i, cv2.CC_STAT_AREA]
            if area < 40: # 24 * 32. before resize. after adjusting this code into tile, change this value if needed.
                continue  # if ignored too much, reduce value. if too many noise, increase value. 

            cx, cy = centroids[i] # this is center before resize. 32*24
            #centers.append( (cx,cy) )
            px = int(cx * scale_x)
            py = int(cy * scale_y) # this is center after resize. 640*480
            # after resize 
            centers.append( (px,py) )
            cv2.circle(vis, (px, py), 8, (0, 255, 255), -1)  # make as yellow dot after resize to visualize.

        
        # started -> capture hot center -> get to cloested area ( rightmost, if wall is on the right )
        # we have to stop when hot center is on the 320 ( center of the camera )
        # just stop when hot center comes to ther 320? 

        
        if centers:
                print(f"num of centers : {len(centers)}") # print number of center of hot area
                rightmost = max(centers, key = lambda p: p[0]) # find rightmost center. find max x coordinate 
                rightmost_x = rightmost[0] # x coordinate of rightmost. what we want. -
                rightmost_y = rightmost[1] # y coordinate of rightmost. not neccessary
                move_x = (320 - rightmost_x) # how much pixel(after resize) should car move? + : move left, - : move right. send signal 
                # from here, we have to send signal to auto drive code. this if just for check
                if (move_x ) > 0 :
                    print("move left")
                elif (move_x ) < 0 :
                    print("move right")
                else:
                    print("stop")
        

        # show max / min temp, fps
        fps  = 1.0 / (time.time() - t0)
        info = f"Tmin={frame.min():+.1f}  Tmax={frame.max():+.1f}  FPS={fps:.2f}"
        cv2.putText(vis, info, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)

        # visualize
        cv2.imshow("Output", vis)

        # if press s -> save image. i don't think it is necessary...
        if (cv2.waitKey(1) & 0xFF) == ord("s"):
            fname = "pic_" + dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + '.jpg'
            cv2.imwrite(fname, vis)
            print("Saved image", fname)

except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("Stopped")

cv2.destroyAllWindows()
