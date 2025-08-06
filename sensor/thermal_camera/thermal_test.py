'''
short explanation

this code find hot area using gradient. maybe it can be controlled by adjusting value of threshold whatever you want. << ask gpt...
after it find hot place, it caculate it's center and center is saved to cx, cy. ( cx, cy is coordinate of "before" resized. px, py is after resized.)
after it find center

1) find rightmost of center comparing x coordinate
2) caculate how much should car move

area 변경필요! -> 영역의 최소값을 정의할 필요가 있다!
'''



import time, board, busio
import numpy as np
import adafruit_mlx90640
import datetime as dt
import cv2

# === Constants ===
SHAPE = (24, 32)
RESIZED = (640, 480)
THRESHOLD = 1.5
AREA_MIN = 40  # 최소 면적 조건
SCALE_X = RESIZED[0] / SHAPE[1]
SCALE_Y = RESIZED[1] / SHAPE[0]


def setup_mlx():
    """MLX90640 초기 설정"""
    i2c = busio.I2C(board.SCL, board.SDA, frequency=1200000)
    mlx = adafruit_mlx90640.MLX90640(i2c)
    mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_16_HZ
    return mlx


def temp_to_u8(arr, vmin=22.0, vmax=36.0):
    """온도값을 8비트 그레이스케일로 변환"""
    clipped = np.clip(arr, vmin, vmax)
    return ((clipped - vmin) * (255.0 / (vmax - vmin))).astype(np.uint8).reshape(SHAPE)


def compute_gradient_mask(img_f32, threshold=THRESHOLD):
    """온도 gradient로부터 hot area 마스크 생성"""
    gy, gx = np.gradient(img_f32)
    grad_mag = np.sqrt(gx**2 + gy**2)
    mask = (grad_mag > threshold).astype(np.uint8)
    return mask


def clean_mask(mask):
    """마스크에 morphology 연산 적용 (노이즈 제거 및 클러스터 연결)"""
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k, iterations=1)
    return mask


def find_hot_centers(mask, stats_area_min=AREA_MIN):
    """마스크에서 중심좌표를 찾아 리스트로 반환"""
    centers = []
    n_labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

    for i in range(1, n_labels):  # 0은 background
        area = stats[i, cv2.CC_STAT_AREA]
        if area < stats_area_min:
            continue
        cx, cy = centroids[i]
        px = int(cx * SCALE_X)
        py = int(cy * SCALE_Y)
        centers.append((px, py))
    return centers


def draw_centers(vis, centers):
    """시각화용 중심점 그리기"""
    for (px, py) in centers:
        cv2.circle(vis, (px, py), 8, (0, 255, 255), -1)


def determine_move(centers):
    """중심점 중 오른쪽 끝 좌표를 기준으로 이동 방향 계산"""
    if not centers:
        return None

    rightmost = max(centers, key=lambda p: p[0])
    move_x = 320 - rightmost[0]  # 중심 기준으로 차이가 얼마나 나는지

    if move_x > 0:
        print("move left")
    elif move_x < 0:
        print("move right")
    else:
        print("stop")
    return move_x


def show_info(vis, frame, fps):
    """최소/최대 온도 및 FPS 시각화"""
    info = f"Tmin={frame.min():+.1f}  Tmax={frame.max():+.1f}  FPS={fps:.2f}"
    cv2.putText(vis, info, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)


def main():
    mlx = setup_mlx()
    frame = np.zeros(24 * 32, dtype=np.float32)
    print("code start")
    time.sleep(4)

    try:
        while True:
            t0 = time.time()

            # === Get Frame and Preprocess ===
            mlx.getFrame(frame)
            img_f32 = frame.reshape(SHAPE)
            u8_img = temp_to_u8(img_f32)
            vis = cv2.applyColorMap(u8_img, cv2.COLORMAP_JET)
            vis = cv2.resize(vis, RESIZED, cv2.INTER_CUBIC)

            # === Hotspot Detection ===
            mask = compute_gradient_mask(img_f32)
            mask = clean_mask(mask)
            centers = find_hot_centers(mask)

            # === Draw and Decide Move ===
            draw_centers(vis, centers)
            determine_move(centers)

            # === Info and Visualization ===
            fps = 1.0 / (time.time() - t0)
            show_info(vis, frame, fps)
            cv2.imshow("Output", vis)

            # Save Image
            if (cv2.waitKey(1) & 0xFF) == ord("s"):
                fname = "pic_" + dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + '.jpg'
                cv2.imwrite(fname, vis)
                print("Saved image", fname)

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print("Stopped")


if __name__ == "__main__":
    main()
