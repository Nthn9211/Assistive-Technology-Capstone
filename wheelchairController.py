#!/usr/bin/env python
import rospy
from wheelchair.msg import gaze_msg
import RPi.GPIO as GPIO   # Import the GPIO library.
import time               # Import time library

GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.
                          # Can use GPIO.setmode(GPIO.BCM) instead to use
                          # Broadcom SOC channel names.
#  MBTechWorks.com 2016
#  Pulse Width Modulation (PWM) demo to cycle brightness of an LED
GPIO.setwarnings(False)

GPIO.setup(16, GPIO.OUT)  # Set GPIO pin 16 to output mode. Blue wire
GPIO.setup(18, GPIO.OUT)  # Set GPIO pin 18 to output mode. Orange wire
GPIO.setup(11, GPIO.IN)  # Set GPIO pin 11 to input mode. Coordinate X white wire
GPIO.setup(13, GPIO.IN)  # Set GPIO pin 11 to input mode. Angular Z  black wire
bluePin = GPIO.PWM(18, 100)   # Initialize PWM on pwmPin 100Hz frequency
orangePin = GPIO.PWM(16, 100)   # Initialize PWM on pwmPin 100Hz frequency

MAXPWM = 92
MINPWM = 62

bluePWM=0     # set blue PWM variable
orangePWM=0    # set orange PWM variable

bluePin.start(bluePWM) #Start PWM on blue wire
orangePin.start(orangePWM) #Start PWM on orange wire
bluePin.ChangeDutyCycle(bluePWM)
orangePin.ChangeDutyCycle(orangePWM)

class WheelchairController:

    sampleCountAngZ = 0
    sampleCountLinX = 0
    totalXVal = 0
    totalZVal = 0

    def __init__(self):
        rospy.init_node('wheelChairListener', anonymous=True)
        rospy.Subscriber("gaze_info", gaze_msg, self.callback)

    def callback(self, data):
        self.incrementLinX(data.lin_x)
        self.incrementAngZ(data.ang_z)

        if(self.sampleCountAngZ >= 300):
            x = self.averageLinX()
            z = self.averageAngZ()
            print(x,z)
            self.pwmHandler(x, z)

    def pwmHandler(self, lin_x, ang_z):
        #global bluePWM
        #global orangePWM

        # Conversion from linear input to exponential PWM output
        bluePWM = 100 * (((0.16) * ((lin_x / 10) ** 3)) + 0.78)
        orangePWM = 100 * (((0.16) * ((ang_z / 10) ** 3)) + 0.78)

        # Ensuring PWM does not exceed limits
        if (bluePWM > MAXPWM):
            bluePWM = MAXPWM

        if (orangePWM > MAXPWM):
            orangePWM = MAXPWM

        if (bluePWM < MINPWM):
            bluePWM = MINPWM

        if (orangePWM < MINPWM):
            orangePWM = MINPWM

        bluePin.ChangeDutyCycle(bluePWM)  # Updating PWM for linear x
        orangePin.ChangeDutyCycle(orangePWM)  # Updating PWM for angular z

    def incrementLinX(self, sample):
        self.sampleCountLinX += 1
        self.totalXVal += sample

    def incrementAngZ(self, sample):
        self.sampleCountAngZ += 1
        self.totalZVal += sample

    def averageAngZ(self):
        average = self.totalZVal / self.sampleCountAngZ
        self.sampleCountAngZ = 0
        self.totalZVal = 0
        return average

    def averageLinX(self):
        average = self.totalXVal / self.sampleCountLinX
        self.sampleCountAngX = 0
        self.totalXVal = 0
        return average

if __name__ == '__main__':
    WCController = WheelchairController()
    rospy.spin()
