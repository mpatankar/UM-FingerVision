import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

p = GPIO.PWM(18, 4231)

p.start(0)

p.ChangeDutyCycle(100)

try:
	while True:
		time.sleep(0.02)
		
except KeyboardInterrupt:
	p.stop()
	GPIO.cleanup()
