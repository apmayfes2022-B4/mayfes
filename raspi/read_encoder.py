# ロータリーエンコーダを読むプログラム

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

pinA = 20
pinB = 21

GPIO.setup(6, GPIO.OUT)
GPIO.setup(pinA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.output(6, True)

count = 0

def callback(pinA):
  global count
  if (GPIO.input(pinA) == GPIO.input(pinB)):
    count = count + 1
  else:
    count = count - 1

GPIO.add_event_detect(pinA, GPIO.BOTH)
GPIO.add_event_callback(pinA, callback)

for _ in range(20):
  print(count)
  time.sleep(1)

GPIO.cleanup()

