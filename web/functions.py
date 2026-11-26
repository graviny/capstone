#!/usr/bin/env python3
# File name   : functions.py
# Website     : www.adeept.com
# Author      : Adeept
# Date        : 2025/05/15
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

import threading
import os
import json
import ultra
import Kalman_filter
import move
import RPIservo
import smbus
from gpiozero import InputDevice
last_status = 0


lightADC = 127
lightThreshold = 15


line_pin_left = 22
line_pin_middle = 27
line_pin_right = 17

class ADS7830(object):
    def __init__(self):
        self.cmd = 0x84
        self.bus=smbus.SMBus(1)
        self.address = 0x48 # 0x48 is the default i2c address for ADS7830 Module.   
        
    def analogRead(self, chn): # ADS7830 has 8 ADC input pins, chn:0,1,2,3,4,5,6,7
        value = self.bus.read_byte_data(self.address, self.cmd|(((chn<<2 | chn>>1)&0x07)<<4))
        return value

adc = ADS7830()

scGear = RPIservo.ServoCtrl()
scGear.start()


move.setup()
kalman_filter_X =  Kalman_filter.Kalman_filter(0.01,0.1)


curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

def num_import_int(initial):        #Call this function to import data from '.txt' file
	global r
	with open(thisPath+"/RPIservo.py") as f:
		for line in f.readlines():
			if(line.find(initial) == 0):
				r=line
	begin=len(list(initial))
	snum=r[begin:]
	n=int(snum)
	return n

pwm0_direction = 1
pwm0_init = num_import_int('init_pwm0 = ')
pwm0_max  = 180
pwm0_min  = 0
pwm0_pos  = pwm0_init

pwm1_direction = 1
pwm1_init = num_import_int('init_pwm1 = ')
pwm1_max  = 180
pwm1_min  = 0
pwm1_pos  = pwm1_init

pwm2_direction = 1
pwm2_init = num_import_int('init_pwm2 = ')
pwm2_max  = 180
pwm2_min  = 0
pwm2_pos  = pwm2_init

line_pin_left = 22
line_pin_middle = 27
line_pin_right = 17


class Functions(threading.Thread):
	def __init__(self, *args, **kwargs):
		self.functionMode = 'none'
		self.steadyGoal = 0

		self.scanNum = 3
		self.scanList = [0,0,0]
		self.scanPos = 1
		self.scanDir = 1
		self.rangeKeep = 30
		self.scanRange = 100
		self.scanServo = 1
		self.turnServo = 2
		self.turnWiggle = 200

		super(Functions, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()
		self.__flag.clear()



	def pwmGenOut(self, angleInput):
		return int(angleInput)

	def setup(self):
		global track_line_left, track_line_middle,track_line_right
		track_line_left = InputDevice(pin=line_pin_left)
		track_line_middle = InputDevice(pin=line_pin_middle)
		track_line_right = InputDevice(pin=line_pin_right)

	def radarScan(self):
		pwm0_min = -90
		pwm0_max =  90

		scan_speed = 1
		result = []

		pwm0_pos = pwm0_max
		scGear.moveAngle(1, 0)
		time.sleep(0.8)

		while pwm0_pos>pwm0_min:
			pwm0_pos-=scan_speed
			scGear.moveAngle(1, pwm0_pos)
			dist = ultra.checkdist()
			if dist > 50:
				continue
			theta = 90 - pwm0_pos 
			result.append([dist, theta])
			time.sleep(0.02)
	
		scGear.moveAngle(1, 0)
		return result


	def pause(self):
		self.functionMode = 'none'
		move.motorStop()
		self.__flag.clear()


	def resume(self):
		self.__flag.set()


	def automatic(self):
		self.functionMode = 'Automatic'
		self.resume()


	def trackLine(self):
		self.functionMode = 'trackLine'
		self.resume()


	def keepDistance(self):
		self.functionMode = 'keepDistance'
		self.resume()

	def trackLight(self):
		self.functionMode = 'trackLight'
		self.resume()



	def trackLineProcessing(self):
		global last_status
     
		status_right = track_line_right.value
		status_middle = track_line_middle.value
		status_left = track_line_left.value
		current_status = (status_left << 2) | (status_middle << 1) | status_right

		if last_status == current_status:
			return

		last_status = current_status

		if status_middle == 0:
			if status_left == 0 and status_right == 1:    # 0 0 1   right
				scGear.moveAngle(0, -38)
				move.move(28,1,"right")
			elif status_left == 1 and status_right == 0:  # 1 0 0 left
				scGear.moveAngle(0, 38)
				move.move(28,1,"left")
			else:									 # 0 0 0 or 1 0 1
				scGear.moveAngle(0, 0)  
				move.move(28,1,"mid")
		else:
			if status_left == 0 and status_right == 1:	#011
				scGear.moveAngle(0, -38)
				move.move(28,1,"right")
			elif status_left == 1 and status_right == 0:	#110
				scGear.moveAngle(0, 38)
				move.move(28,1,"left")
			else:	#010 or 111
				scGear.moveAngle(0, 0)
				move.move(28,1,"mid")
		print(status_left,status_middle,status_right)
		time.sleep(0.1)

	def distRedress(self): 
		mark = 0
		distValue = ultra.checkdist()
		while True:
			distValue = ultra.checkdist()
			if distValue > 200:
				mark +=  1
			elif mark > 5 or distValue <200:
					break
		return round(distValue,2)

	def automaticProcessing(self):
		print('automaticProcessing')
		dist = self.distRedress()
		print(dist, "cm")
		if dist > 60:			# More than 50CM, go straight.
			scGear.moveAngle(0, 0)
			move.move(50, 1, "mid")
			time.sleep(0.3)
			print("Forward")
		elif dist >= 40 and dist <= 60:	 # More than 40cm and less than 60cm, detect the distance between the left and right sides.
			move.move(0, 1, "mid")
			scGear.moveAngle(1, 60)
			time.sleep(0.5)
			distLeft = self.distRedress()
			self.scanList[0] = distLeft

			scGear.moveAngle(1, -60)
			time.sleep(0.5)
			distRight = self.distRedress()
			self.scanList[1] = distRight

			print(self.scanList)
			scGear.moveAngle(1, 0)
			if self.scanList[0] >= self.scanList[1]:
				scGear.moveAngle(0, 40)
				move.move(50, 1, "left")
				time.sleep(0.6)
				print("Left")
			else:
				scGear.moveAngle(0, -40)
				move.move(50, 1, "right")
				time.sleep(0.6)
				print("Right")
		else:		# The distance is less than 40cm, back.
			scGear.moveAngle(0, 0)
			move.move(50, -1, "mid")
			time.sleep(0.3)
			print("Back")
		


	def keepDisProcessing(self):
		distanceGet = self.distRedress()
		print('keepDistanceProcessing: ' + str(distanceGet))
		if distanceGet > 40:
			move.move(40, 1, "mid")
		elif distanceGet < 30:
			move.move(40, -1, "mid")
		else:
			move.motorStop()
		time.sleep(0.3)
   
	def trackLightProcessing(self):
		global last_status

		adc_value = adc.analogRead(1)  
		if last_status == 0:
			pass
		elif adc_value < lightADC - lightThreshold and last_status < lightADC - lightThreshold:
			return
		elif adc_value > lightADC + lightThreshold and last_status > lightADC + lightThreshold:
			return
		elif adc_value > lightADC - lightThreshold and adc_value < lightADC + lightThreshold and last_status > lightADC - lightThreshold and last_status < lightADC + lightThreshold:
			return
		last_status = adc_value
  
		print(f"Light Tracking Value: {adc_value}")
		if adc_value < lightADC - lightThreshold:
			scGear.moveAngle(0, 40)
			move.move(50, 1, "left")
		elif adc_value > lightADC + lightThreshold:
			scGear.moveAngle(0, -40)
			move.move(50, 1, "right")
		else:
			move.move(30,1,"mid")
		time.sleep(0.2)


	def functionGoing(self):
		if self.functionMode == 'none':
			self.pause()
		elif self.functionMode == 'Automatic':
			self.automaticProcessing()
		elif self.functionMode == 'trackLine':
			self.trackLineProcessing()
		elif self.functionMode == 'keepDistance':
			self.keepDisProcessing()
		elif self.functionMode == 'trackLight':
			self.trackLightProcessing()


	def run(self):
		while 1:
			self.__flag.wait()
			self.functionGoing()
			pass


if __name__ == '__main__':
	pass
	try:
		fuc=Functions()
		fuc.setup()
		while True:
			fuc.keepDisProcessing()
	except KeyboardInterrupt:

			move.motorStop()
