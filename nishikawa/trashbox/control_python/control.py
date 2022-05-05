import numpy as np
import RPi.GPIO as GPIO
import time

from ....kato.trashbox.header import EKF, Encoder # パスは適宜書き換えてください。

def get_start():
    return 0, 0

def get_target():
    return 500, 500

def motorpin_setup(MOTOR_PIN, pwm):
    for pin in MOTOR_PIN.keys():
        GPIO.setup(MOTOR_PIN[pin], GPIO.OUT)
        pwm[pin] = GPIO.PWM(MOTOR_PIN[pin], 60)
        pwm[pin].start(0)

def encoderpin_setup(ENCODER_PIN):
    for pin in ENCODER_PIN.value():
        GPIO.setup(pin, GPIO.IN)

def encoder_callback(pinA, pinB, i, encoder_val):
    """
    pinA, pinB: ピン番号
    i: エンコーダの番号
    """
    if (GPIO.input(pinA) == GPIO.input(pinB)):
        encoder_val[i-1] += 1
    else:
        encoder_val[i-1] -= 1


def PI_control_direction(theta, theta_sum, Kp_direction, Ki_direction):
    theta_sum += theta
    direction_correct = Kp_direction * (0-theta) + Ki_direction * theta_sum

    # 100より大きい場合はエラーを吐く。この場合、Kp_direction や Ki_direction を調整する。
    if direction_correct > 100:
        raise Exception("`direction_correct` is too large.")

    return direction_correct


def P_control_velocity(delta_x, delta_y, Kp_velocity, direction_correct):
    motor_val = [0]*3
    # P制御
    motor_val[0] = Kp_velocity * delta_y;
    motor_val[1] = Kp_velocity * ((3**0.5) * delta_x - delta_y) / 2;
    motor_val[2] = Kp_velocity * (- (3**0.5) * delta_x - delta_y) / 2;

    # 絶対値が100を超えてしまう場合は、100を超えないギリギリにする。結果、最初の方はbangbang制御になる。
    # ここで決めた出力にdirection_correctが加算されることに注意する．
    available_min = - 100 - direction_correct
    available_max = 100 - direction_correct
    min_motor_val = min(motor_val)
    max_motor_val = max(motor_val)
    if (min_motor_val < available_min or max_motor_val > available_max):
        bangbang_ratio = min(available_max/max_motor_val, available_min/min_motor_val)
        for i in range(3):
            motor_val[i] = bangbang_ratio * motor_val[i]
    return motor_val

def motor_output(pwm, motor_val):
    for i in (1,2,3):
        pwm[f"{i}A"].ChangeDutyCycle(max(motor_val[i-1], 0))
        pwm[f"{i}B"].ChangeDutyCycle(max(-motor_val[i-1], 0))

def control(enc, ekf, Kp_direction=1, Ki_direction=1, Kp_velocity=1,
        MOTOR_PIN = {'1A': 23, '1B': 24, '2A':25, '2B':12, '3A':17, '3B':27}, 
        ENCODER_PIN = {'1A': 20, '1B': 21, '2A':19, '2B':26, '3A':9, '3B':11}):
    """
    推定するための関数等の呼び出しからモーターへの出力まで行う
    カメラ等の情報による

    input: enc, ekf, Kp_direction, Ki_direction, Kp_velocity, MOTOR_PIN, ENCODER_PIN\\
    enc, ekf はそれぞれエンコーダー、自己位置推定をするオブジェクト\\
    Kp_direction, Ki_direction, Kp_velocity は角度のPI制御、および速さのP制御に関するパラメータ\\
    MOTOR_PIN, ENCODER_PIN はそれぞれピン番号、HackMdにかいてあるやつをデフォルトにしている。

    output: None
    """
    encoder_val = [0]*3 # 各エンコーダの値を格納
    theta_sum = 0 # これまでのthetaの和 制御(I)に用いる
    pwm = {x:0 for x in MOTOR_PIN.keys()}  # PWMを調整するオブジェクトたちを管理

    # setup
    motorpin_setup(MOTOR_PIN, pwm)
    encoderpin_setup(ENCODER_PIN)

    # 各エンコーダについて、pinAの立ち上がりもしくは立ち下がりが起こったときに検知し、callbackで指定されたコールバック関数を呼び出す
    for i in (1,2,3):
        GPIO.add_event_detect(ENCODER_PIN[f'{i}A'], GPIO.BOTH, callback=lambda x:encoder_callback(x, ENCODER_PIN[f'{i}B'], i, encoder_val))

    # 時間に関する変数　enc, ekfで利用する
    t_now = time.time()
    t_before = 0
    t_diff = 0.01

    while True:
        # 毎ループ、スタート地点とゴール地点を取得
        # 呼び出し時の引数にゴール地点が与えられていて、その近くに達したら終了するような関数にしたほうがよい？
        start_x, start_y = get_start()
        target_x, target_y = get_target()
        delta_x = target_x - start_x
        delta_y = target_y - start_y

        old_encoder_val = encoder_val[:]

        # 時間に関する変数　enc, ekfで利用する
        t_before = t_now
        t_now = time.time()
        t_diff = t_now - t_before

        # エンコーダから速度と角速度を求める
        enc.set_t_diff(t_diff)
        enc.update(encoder_val)
        omega = enc.Omega() # これがほしいっぽい

        # エンコーダから得た値を使って現在位置と現在角度が更新される。
        ekf.set_t_diff(t_diff)
        y_now = np.vstack((np.array([[start_x],[start_y]]),omega))
        ekf.update(0, y_now) # self.xが更新(第0,1要素は座標、第2要素はtheta)

        # PI制御を用いて向きを調整
        direction_correct = PI_control_direction(ekf.x[2], theta_sum, Kp_direction, Ki_direction)
        
        # P制御を用いて各モーターのduty比を決定
        motor_val = P_control_velocity(delta_x, delta_y, Kp_velocity, direction_correct)

        # モーター出力
        motor_output(pwm, motor_val)


if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    enc = Encoder.Encoder() # エンコーダの値から速度や角速度を出してくれるオブジェクト
    ekf = EKF.EKF() # 推定をやってくれるオブジェクト
