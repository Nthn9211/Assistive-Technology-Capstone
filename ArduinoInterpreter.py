#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import Float32MultiArray
from falcongaze.msg import gaze_msg
from geometry_msgs.msg import Twist
from time import time
import serial
import sys

# Constants
MAXPWM = 155 #Maximum duty cycle value for high voltage output
MINPWM = 101 #Minimum duty cycle value for low voltage output
CONFTHRESH = 0.7 #confidence threshold for acceptable user input
TIMETHRESH = 3 #number of seconds to close eyes for shutdown

# Global vars for pass-through (Only used for ROSBot testing)
linXPassThrough = 0
angZPassThrough = 0

# Global flag
armed = False
baseTime = 0 #Stores value of a base time for comparison for shutdown procedure

# Initializing PWM variables
bluePWM = 0  # blue (fwd/back)
orangePWM = 0  # orange (left/right)

# Setting up serial port for communication with Arduino
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.flush()

# Creating a new gaze message for publishing to in twist format
rosBotMessage = Twist()

def armedCheck(data):
    global armed

    if armed:
        # print("Armed")
        if data.surface == 0: #If surface is not recognized, disarm
            armed = False

    if not armed:
        #print("Disarmed")
        if data.surface == 1: #If surface recognized and user looking in deadzone, arm
            if deadzone(data):
                armed = True


def deadzone(data):
    global deadzone
    if -0.2 < data.lin_x < 0.2 and -0.2 < data.ang_z < 0.2: #If in in deadzone, return true (we are in deadzone)
        return True
    else:
        return False


def shutdown(data): #Checks for shutdown
    global baseTime
    tempTime = 0

    if data.conf_0 >= CONFTHRESH and data.conf_1 >= CONFTHRESH: # Continuously set a base time value while confidence is good
        baseTime = time() #Gets current time and stores it in base time var
    elif data.conf_0 < 0.15 and data.conf_1 >= CONFTHRESH:  # Right eye closed, left eye open for shutdown
        tempTime = time() # If only right eye closed, set new temp variable for comparison
        if tempTime - baseTime > TIMETHRESH: #If eye closed for TIMETHRESH amount of time, exit and shutdown NUC
           print("Exiting!")
           os.system("shutdown now")


def callback(data):
    global bluePWM  # Linear X Pin
    global orangePWM  # Angular Z Pin
    global linXPassThrough
    global angZPassThrough
    global armed

    # Checking for valid input (is the system armed or disarmed)
    armedCheck(data)

    # Checking timer for shutdown and check that we are in deadzone
    shutdown(data)
    deadzone(data)

    if armed and (data.conf_0 >= CONFTHRESH and data.conf_1 >= CONFTHRESH) and data.surface == 1:
        # Conversion from linear input to exponential PWM output (depreciated - left in for documentation)
        #bluePWM = ((43 * (data.lin_x ** 3)) + 128)
        #orangePWM = ((43 * ((-1 * data.ang_z) ** 3)) + 128)
        
        # Mapping input coordinates to linear function for duty cycles (0 - 255 value on arduino)
        bluePWM = (27 * data.lin_x) + 128 #Intercept of 128 gives us the 'centered' voltage to send to arduino
        orangePWM = 27 * (-1 * data.ang_z) + 128 #slope can be adjusted to make steering more or less aggressive (Higher slope = more aggressive)

        # Ensuring PWM does not exceed limits
        if bluePWM > MAXPWM:
            bluePWM = MAXPWM
        if orangePWM > MAXPWM:
            orangePWM = MAXPWM
        if bluePWM < MINPWM:
            bluePWM = MINPWM
        if orangePWM < MINPWM:
            orangePWM = MINPWM
            
    #if not armed, set to default value (no movement)
    elif not armed:
        bluePWM = 128 
        orangePWM = 128 

    # Passing Through Lin X and Ang Z Values to cmd_vel from gaze_msg (for ROSbot)
    #linXPassThrough = data.lin_x
    #angZPassThrough = data.ang_z

    # Writing Serial values to Arduino
    # Current Message: BBBOOOs (Blue pin duty cycle value(BBB), Orange pin duty cycle value (OOO), Terminating character(s))
    bluePWM = round(bluePWM) #cutting off decimal values
    orangePWM = round(orangePWM)
    testString = str(bluePWM) + str(orangePWM) + "s" #terminating character can be changed to whatever you want
    ser.write(testString.encode('utf-8')) #publishing to arduino serially
    print(testString)
    print(data.conf_0, data.conf_1)

    # Publishing values to cmd_vel to control ROSBot
    #rosBotMessage.linear.x = linXPassThrough
    #rosBotMessage.angular.z = angZPassThrough
    #pub_vel.publish(rosBotMessage)

    # Print values for debugging
    # print(bluePWM, orangePWM)
    # print(data.ang_z)
    # print(testString)


if __name__ == '__main__':
    rospy.init_node('ArduinoInterpreter_ROSBot', anonymous=True)
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #Setting up publisher for ROSBot control
    rospy.Subscriber("/gaze_info", gaze_msg, callback) #Setting up subscriber for listening to Pupil Glasses
    rospy.spin() #Repeats until ros network shutsdown
