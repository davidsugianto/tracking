import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
Pins=[9,10,18,14,15,17,23,24,25,12,16,20,21]
for i in Pins:
	GPIO.setup(i,GPIO.OUT)
pwm0 = GPIO.PWM(18,100)
pwm1 = GPIO.PWM(17,100)
pwm2 = GPIO.PWM(25,100)
pwm0.start(0)
pwm1.start(0)
pwm2.start(0)
Low = GPIO.LOW
High = GPIO.HIGH

def motor_belakang(cw,ccw,pwm_be):
	GPIO.output(9,cw)
	GPIO.output(10,ccw)
	pwm0.ChangeDutyCycle(pwm_be)

def motor_kanan(cw,ccw,pwm_ka):
	GPIO.output(15,cw)
	GPIO.output(14,ccw)
	pwm1.ChangeDutyCycle(pwm_ka)

def motor_kiri(cw,ccw,pwm_ki):
	GPIO.output(23,cw)
	GPIO.output(24,ccw)
	pwm2.ChangeDutyCycle(pwm_ki)

def maju():
	motor_kanan(1,0,15)
	motor_kiri(1,0,15)
	#motor_belakang(0,0,30)

def mundur():
	motor_kanan(0,1,15)
	motor_kiri(0,1,15)

def kanan():
	motor_kanan(0,1,15)
	motor_kiri(1,0,10)
	motor_belakang(1,0,15)
	
def kiri():
	motor_kanan(1,0,10)
        motor_kiri(0,1,15)
        motor_belakang(0,1,15)

def putar_kanan():
        motor_kanan(1,0,20)
        motor_kiri(0,1,20)
        motor_belakang(1,0,20)

def putar_kiri():
        motor_kanan(0,1,20)
        motor_kiri(1,0,20)
        motor_belakang(0,1,20)

	
while True:
	maju
