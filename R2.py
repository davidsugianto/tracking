import RPi.GPIO as GPIO
import cv2
import numpy as np
import sys
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
cap = cv2.VideoCapture(0)
cap.set(3,320)
cap.set(4,240)
cap.set(15,0.1)
simpan_data = 0
data_x = 0 		#posisi bola
data_y = 0 		#jarak bola dengan robot
data_radius = 0	#ukuran bola
#variable PID controller
x=0
y=0
radius=0
Pid = 0
error = 0 
last_error = 0
Kp = 12
Ki = 1
Kd = 20
Ts = 1
fast_ka = float(20)
fast_ki = float(20)
fast_be = 0
max_speed = 20
min_speed = 0
set_point = 30
pwm_be = 0
#


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
	motor_kanan(1,0,40)
	motor_kiri(1,0,40)

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

def putar_kiri():
        motor_kanan(1,0,15)
        motor_kiri(0,1,15)
        motor_belakang(1,0,15)

def putar_kanan():
        motor_kanan(0,1,15)
        motor_kiri(1,0,15)
        motor_belakang(0,1,15)
        
def stop():
        motor_kanan(0,0,20)
        motor_kiri(0,0,20)
        motor_belakang(0,0,20)

def tracking():
	global radius
	global x
	global y
	_,video = cap.read()
	video = cv2.flip(video,0)
	data = np.array([0,0,0])
	hsv = cv2.cvtColor(video,cv2.COLOR_BGR2HSV)
	lower_orange = np.array([0,98,113])
	upper_orange = np.array([66,175,251])
	kernel = np.ones((5,5),np.uint8)
	colors = (0,140,255)
	mask = cv2.inRange(hsv, lower_orange, upper_orange)
	dilation = cv2.dilate(mask, kernel,  iterations = 1)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	mask = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
	mask = cv2.GaussianBlur(mask,(5,5),0)
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None
	
	if len(cnts) > 0:
		c = max(cnts, key=cv2.contourArea)
		((x,y),radius) = cv2.minEnclosingCircle(c)
		data = np.array([int(x), int(y), int(radius)])
		#if radius > 10:
			#cv2.circle(video, (int(x), int(y)), int radius(), colors,2)
	else:
		data = np.array([0,0,0])
	print'(%d,%d,%d)'%(x,y,radius)	
	return data
	
def pid(pv,sp):
	global error
	global last_error
	error=pv-sp
 	hasil_PID=float(((Kp*error) +((Ki/10)*(error+last_error)*Ts)+((Kd/Ts)*(error-last_error))))
	last_error=error
	return hasil_PID
	print"%d,%d,%d"%(int(error),int(last_error),int(hasil_PID))

def main():
	jarak_bola = tracking()
	pv = jarak_bola[0]
	PID = pid(pv,160)
	pwm_ka = fast_ka+PID
	pwm_ki = fast_ki-PID
	if (pwm_ka > fast_ka+15):
		pwm_ka=fast_ka+15
	elif (pwm_ka < min_speed):
		pwm_ka=min_speed
	if (pwm_ki > fast_ki+15):
		pwm_ki=fast_ki+15
	elif (pwm_ki < min_speed):
		pwm_ki=min_speed
	if pwm_ki >= (fast_ki+20):
		motor_belakang(1,0,20)
	elif pwm_ka >= (fast_ka+20):
		motor_belakang(0,1,20)
	else:
		motor_belakang(0,0,0)
	motor_kanan(1,0,pwm_ka)
	motor_kiri(1,0,pwm_ki)
	print "%d, %d"%(int(pwm_ka),int(pwm_ki))
	
while True:
	main()

