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

bluePWM=0     # set blue PWM variable
orangePWM=0    # set orange PWM variable
MAXPWM = 92
MINPWM = 62

bluePin.start(bluePWM) #Start PWM on blue wire
orangePin.start(orangePWM) #Start PWM on orange wire
bluePin.ChangeDutyCycle(bluePWM)
orangePin.ChangeDutyCycle(orangePWM)

WheelchairAverager = Averager()

def callback(data):

    WheelchairAverager.incrementXTotal(data.lin_x)
    WheelchairAverager.incrementZTotal(data.ang_z)
    
    if(WheelchairAverager.sampleCountAngZGetter >= 1000):
        x = WheelchairAverager.averageLinX()
        z = WheelchairAverager.averageAngZ()
        pwmHandler(x, z)

    print(bluePWM)

def pwmHandler(lin_x, ang_z):
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

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('wheelChairListener', anonymous=True)

    rospy.Subscriber("gaze_info", gaze_msg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

class Averager:
    sampleCountAngZ = 0
    sampleCountLinX = 0
    totalXVal = 0
    totalZVal = 0

    def incrementLinX(sample):
        self.sampleCountLinX += 1
        self.totalXVal += sample

    def incrementAngZ(sample):
        self.sampleCountAngZ += 1
        self.totalZVal += sample

    def sampleCountAngZGetter():
        return self.sampleCountAngZ

    def sampleCountLinXGetter():
        return self.sampleCountLinX

    def averageAngZ():
        average = self.totalZVal/self.sampleCountAngZ
        self.sampleCountAngZ = 0
        self.totalZVal = 0
        return average

    def averageLinX(angXSample):
        average = self.totalXVal/self.sampleCountLinX
        self.sampleCountAngX = 0
        self.totalXVal = 0
        return average

if __name__ == '__main__':
    listener()