#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
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

bluedc=80                              # set blue dc variable to 0 for 0%
orangedc=80
bluePin.start(bluedc)
orangePin.start(orangedc)
bluePin.ChangeDutyCycle(bluedc)
orangePin.ChangeDutyCycle(orangedc)

maxPWM = 92

def callback(data):
    
    bluedc = 100*(((0.16)*((data.linear.x/10)**3)) + 0.78) #Conversion from linear input to exponential PWM output
    orangedc = 100*(((0.16)*((data.angular.z/10)**3)) + 0.78)

    #Ensuring PWM does not exceed max allowable
    if(bluedc > maxPWM):
        bluedc = maxPWM
        
    if(orangedc > maxPWM):
        orangedc = maxPWM

    bluePin.ChangeDutyCycle(bluedc) #Updating PWM for linear x
    orangePin.ChangeDutyCycle(orangedc) #Updating PWM for angular z

    print(bluedc)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('wheelChairListener', anonymous=True)

    rospy.Subscriber("cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()