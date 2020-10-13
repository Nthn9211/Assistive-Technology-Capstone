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




cmd_vel = Twist()   # uses the ros bot x,y,z




def talker():   # publisher

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    cmd_vel.linear.x = 0.5
    
    cmd_vel.linear.z = 0.0



    while not rospy.is_shutdown():

        irsensor()   # acts like an interrupt if the sensors detect something

        pub.publish(cmd_vel)

        rate.sleep()




    bluePin.stop()                         # stop PWM

    orangePin.stop()                         # stop PWM

    GPIO.cleanup()                     # resets GPIO ports used back to input modei to use pin number when referencing GPIO pins.





def irsensor():

 #   time.sleep(0.4)


# note the PWM i have yet to change based on which sensor is detected

    if(GPIO.input(16) == True):     # left sensor detects something go right 

        print("obstacle detected")

        cmd_vel.linear.x = 0 # stop going forward
        
        cmd_vel.linear.z = 0.5 # robot goes right

        orangedc = 76

        orangePin.ChangeDutyCycle(orangedc)

        bluedc = 76

        bluePin.ChangeDutyCycle(bluedc)




    elif (GPIO.input(11) == True): # middle sensor detects back up and go right till it does not detect obstacle

        print("obstacle detected")

        cmd_vel.linear.x = -0.5
        
        cmd_vel.linear.z = 0.5
        
        orangedc = 76

        orangePin.ChangeDutyCycle(orangedc)

        bluedc = 76

        bluePin.ChangeDutyCycle(bluedc)




    elif (GPIO.input(13) == True):  # right sensor detects something go left

        print("obstacle detected")

        cmd_vel.linear.x = 0
    
        cmd_vel.linear.z = -0.5

        orangedc = 76

        orangePin.ChangeDutyCycle(orangedc)

        bluedc = 76

        bluePin.ChangeDutyCycle(bluedc)


    elif (GPIO.input(13) == True):  # if all sensor detect something stop now and back up and go right

        print("obstacle detected")

        cmd_vel.linear.x = -0.5
    
        cmd_vel.linear.z = 0.5
    
        orangedc = 76

        orangePin.ChangeDutyCycle(orangedc)

        bluedc = 76

        bluePin.ChangeDutyCycle(bluedc)



    else:   # if no sensor detects anything then continue straight

        print("no obstacle")

        cmd_vel.linear.x = 0.5 
    
        cmd_vel.linear.z = 0
    
        orangedc = 96.3

        orangePin.ChangeDutyCycle(orangedc)

        bluedc = 100

        bluePin.ChangeDutyCycle(bluedc)



if __name__ == '__main__':

    talker()
    
    
    