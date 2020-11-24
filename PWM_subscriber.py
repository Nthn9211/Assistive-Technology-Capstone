#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from wheelchair.msg import gaze_msg
from wheelchair.msg import AvgValues_msg
import RPi.GPIO as GPIO   # Import the GPIO library.
import time               # Import time library
import serial

GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.

GPIO.setwarnings(False)
GPIO.setup(18, GPIO.OUT)  # Set GPIO pin 18 to output mode. Blue wire
GPIO.setup(16, GPIO.OUT)  # Set GPIO pin 16 to output mode. Orange wire
bluePin = GPIO.PWM(18, 100)   # Initialize blue PWM on pwmPin 100Hz frequency
orangePin = GPIO.PWM(16, 100)   # Initialize orange PWM on pwmPin 100Hz frequency

MAXPWM = 151
MINPWM = 100
THRESH = 0.1
CONFTHRESH = 0

bluePWM=0     # set blue PWM variable
orangePWM=0    # set orange PWM variable

bluePin.start(bluePWM) #Start PWM on blue wire
orangePin.start(orangePWM) #Start PWM on orange wire
bluePin.ChangeDutyCycle(bluePWM)
orangePin.ChangeDutyCycle(orangePWM)

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.flush()

class WheelchairController:

    def __init__(self):

        rospy.init_node('PWMListener', anonymous=True)
        rospy.Subscriber("AvgValues", AvgValues_msg, self.callback)


    def callback(self, data):
        global CONFTHRESH
        global bluePWM
        global orangePWM

        # Conversion from linear input to exponential PWM output
        bluePWM = (((26) * ((data.lin_x) ** 3)) + 125)
        orangePWM = (((26) * (((-1*data.ang_z)) ** 3)) + 125)

        # Ensuring PWM does not exceed limits
        if (bluePWM > MAXPWM):
            bluePWM = MAXPWM

        if (orangePWM > MAXPWM):
            orangePWM = MAXPWM

        if (bluePWM < MINPWM):
            bluePWM = MINPWM

        if (orangePWM < MINPWM):
            orangePWM = MINPWM


        bluePWM = int(bluePWM * 10)
        orangePWM = int(orangePWM * 10)
        print(bluePWM, orangePWM)
        testString = str(bluePWM)+str(orangePWM)+"\n"
        ser.write(testString.encode('utf-8'))
        line = ser.readline().decode('utf-8').rstrip()
        print(line)

        #bluePin.ChangeDutyCycle(bluePWM)  # Updating PWM for linear x
        #orangePin.ChangeDutyCycle(orangePWM)  # Updating PWM for angular z



if __name__ == '__main__':
    WCController = WheelchairController()
    rospy.spin()