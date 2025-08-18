
# # set angle -> duty -> pulse duration 

import Jetson.GPIO as GPIO
import time

# # === 설정 ===
# SERVO_PIN = 33   # BOARD 기준 33번 핀
# FREQ = 50        # 서보모터 주파수 (Hz)

# # === GPIO 초기화 함수 ===
# def setup_servo(pin=SERVO_PIN):
#     GPIO.setwarnings(False)
#     GPIO.setmode(GPIO.BOARD)
#     GPIO.setup(pin, GPIO.OUT)
#     pwm = GPIO.PWM(pin, FREQ)
#     pwm.start(0)
#     return pwm

# # === 각도 설정 함수 ===.
# def set_servo_angle(pwm, angle):
#     duty = 2.5 + (angle / 180.0) * 10  # 2.5~12.5% duty cycle / duty = 2.5 -> 0 degree, 12.5 -> 180degree 
#     pwm.ChangeDutyCycle(duty)
#     time.sleep(0.5)
#     pwm.ChangeDutyCycle(0)  # 신호 끊어 떨림 방지

# # === 종료 함수 ===
# def cleanup_servo(pwm)6:
#     pwm.stop()
#     GPIO.cleanup()



# set angle -> duty -> pulse duration 


# === 설정 ===
SERVO_PIN = 33   # BOARD 기준 33번 핀
FREQ = 50        # 서보모터 주파수 (Hz)

# === GPIO 초기화 함수 ===
def setup_servo(pin=SERVO_PIN):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, FREQ)
    pwm.start(0)
    return pwm

# === 각도 설정 함수 ===.
def set_servo_angle(pwm, angle):
    duty = 2.5 + (angle / 180.0) * 10  # 2.5~12.5% duty cycle / duty = 2.5 -> 0 degree, 12.5 -> 180degree 
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)  # 신호 끊어 떨림 방지

# === 종료 함수 ===
def cleanup_servo(pwm):
    pwm.stop()
    GPIO.cleanup()

