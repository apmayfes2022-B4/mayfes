import RPi.GPIO as GPIO
import sys
import time
"""
encoder_pwm(): (pinA, pinB, duty, 回転方向)を引数としてPWM調整モジュールを作成
start()で開始，
キーボード入力では以下のように調整
"0 e": 前進，"0 d": 行進，"0 q": 停止，"1 (number)": duty=numberとして回転，
関数では rotate('c') ('c' in 'edq') で回転方向変化，duty_change(number)でduty変更．

"""

class encoder_pwm():
    def __init__(self,pinA,pinB,duty=100,direction='e'):
        GPIO.setmode(GPIO.BCM)
        self.duty = duty 
        self.order = direction
        GPIO.setup(pinA, GPIO.OUT)
        GPIO.setup(pinB, GPIO.OUT)        
        self.pA = GPIO.PWM(pinA,60)
        self.pB = GPIO.PWM(pinB,60)

    def start(self,key=False):
        self.pA.start(0)
        self.pB.start(0)
        self.rotate()
        if key:
            try:
                while True:
                    keyin = sys.stdin.readline()[:-1]# 一行をスペース" "で分割
                    order = keyin.strip().split(" ")
                    if order[0] == '0':
                        self.rotate(order[1])
                    elif order[0] == '1':
                        self.duty_change(int(order[1]))
                    else:
                        self.end()
                        break
            except KeyboardInterrupt:
                pass

    def end(self):
        GPIO.cleanup()
        
    def rotate(self,order_new=None):
        #「e」キーが押されたら前進
        if order_new is not None:
            if order_new in 'eds':
                self.order = order_new  
            else:
                raise 'you tried to change direction, but order of  direction is unexpected value'
        
        if self.order == 'e':
            self.pA.ChangeDutyCycle(self.duty)
            self.pB.ChangeDutyCycle(0)
            
        #「d」キーが押されたら後退
        if self.order == 'd':
            self.pA.ChangeDutyCycle(0)
            self.pB.ChangeDutyCycle(self.duty)

        #「q」キーが押されたら止まる
        if self.order == 's':
            self.pA.ChangeDutyCycle(0)
            self.pB.ChangeDutyCycle(0)
    
    def duty_change(self,duty_new):
        if duty_new <= 100:
            self.duty = duty_new
            self.rotate()
        else:
            raise 'you tried to change duty, but number of duty is unexpected value'

# hoge = encoder_pwm(23,24)
# hoge.start()
# count = 0
# while True:
#     count += 1
#     if count == 3:
#         hoge.rotate('d')
#     elif count == 6:
#         hoge.duty_change(50)
#     elif count == 10:
#         hoge.end()
#         break
#     time.sleep(1)
