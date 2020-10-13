#!/usr/bin/env python
import RPi.GPIO as GPIO   # Import the GPIO library.
import time               # Import time library
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


GPIO.setmode(GPIO.BOARD)  # Set Pi to use pin number when referencing GPIO pins.

GPIO.setup(16, GPIO.IN)  # Set GPIO pin 16 to input
GPIO.setup(11, GPIO.IN) # Set GPIO pin 14 to input
GPIO.setup(13, GPIO.IN)  # gpio 15 as input
GPIO.setup(18, GPIO.OUT)  # Set GPIO pin 18 to output mode.
GPIO.setup(15, GPIO.OUT)
bluePin = GPIO.PWM(18, 100)     # Initialize PWM on pwmPin 100Hz frequency
orangePin = GPIO.PWM(15, 100)   # Initialize PWM on pwmPin 100Hz frequency

bluedc=80                              # set blue dc variable to 0 for 0%
orangedc=80
bluePin.start(bluedc)
orangePin.start(orangedc)
bluePin.ChangeDutyCycle(bluedc)
orangePin.ChangeDutyCycle(orangedc)

cmd_vel = Twist()

def talker():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    cmd_vel.linear.x = 0.5

    while not rospy.is_shutdown():
        irsensor()
        pub.publish(cmd_vel)
        rate.sleep()

    bluePin.stop()                         # stop PWM
    orangePin.stop()                         # stop PWM
    GPIO.cleanup()                     # resets GPIO ports used back to input modei to use pin number when referencing GPIO pins.



def irsensor():
 #   time.sleep(0.4)

    if(GPIO.input(16) == True):
        print("obstacle detected")
        cmd_vel.linear.x = 0
        orangedc = 76
        orangePin.ChangeDutyCycle(orangedc)
        bluedc = 76
        bluePin.ChangeDutyCycle(bluedc)

    elif (GPIO.input(11) == True):
        print("obstacle detected")
        cmd_vel.linear.x = 0
        orangedc = 76
        orangePin.ChangeDutyCycle(orangedc)
        bluedc = 76
        bluePin.ChangeDutyCycle(bluedc)

    elif (GPIO.input(13) == True):
        print("obstacle detected")
        cmd_vel.linear.x = 0
        orangedc = 76
        orangePin.ChangeDutyCycle(orangedc)
        bluedc = 76
        bluePin.ChangeDutyCycle(bluedc)

    else:
        print("no obstacle")
        cmd_vel.linear.x = 0.5
        orangedc = 96.3
        orangePin.ChangeDutyCycle(orangedc)
        bluedc = 100
        bluePin.ChangeDutyCycle(bluedc)



if __name__ == '__main__':
    talker()