from nozzle_move import move, close_serial

# 초기 이동 (5초 동안 0, -180으로)
move((0, -180), r_start=0, theta_start=0, duration=5, visualize=False)

# 경로 이동 (linear_speed=3cm/s, 시각화)
path = [
    (2, -180),
    (2.15, -158.2),
    (3.12, -129.81),
    (4.47, -116.57),
    (4.18, -106.7),
    (4.02, -95.71),
    (4.02, -84.29),
    (4.18, -73.3),
    (4.47, -63.43),
    (3.12, -50.19),
    (2.15, -21.8),
    (2.15, 21.8),
    (3.12, 50.19),
    (4.47, 63.43),
    (4.18, 73.3),
    (4.02, 84.29),
    (4.02, 95.71),
    (4.18, 106.7),
    (4.47, 116.57),
    (3.12, 129.81),
    (2.15, 158.2),
    (2, 180)
]
r, theta = move(path, r_start=0, theta_start=-180, linear_speed=3, visualize=True)

# 복귀 이동 (5초 동안 0, 0으로)
move((0, 0), r_start=r, theta_start = theta, duration=5, visualize=False)
close_serial()