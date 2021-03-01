#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from falcongaze.msg import gaze_msg
from geometry_msgs.msg import Twist
from time import time
import serial

# Constants
MAXPWM = 151
MINPWM = 100
THRESH = 0.1
CONFTHRESH = 0

# Global vars for pass-through
linXPassThrough = 0
angZPassThrough = 0

# Global flag
armed = False

# Initializing PWM variables
bluePWM = 0  # blue (fwd/back)
orangePWM = 0  # orange (left/right)

# Setting up serial port for communication with Arduino
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
ser.flush()

# Creating a new gaze message for publishing to
rosBotMessage = Twist()


def pubcheck(data):
    global armed

    if armed:
        #print("Armed")
        if data.surface == 0:
            armed = False

    if not armed:
        #print("Disarmed")
        if data.surface == 1:
            if deadzone(data):
                armed = True


def deadzone(data):
    if -0.2 < data.lin_x < 0.2 and -0.2 < data.ang_z < 0.2:
        #print("In Deadzone!")
        return True
    else:
        return False


def callback(data):
    global bluePWM  # Linear X Pin
    global orangePWM  # Angular Z Pin
    global linXPassThrough
    global angZPassThrough
    global armed

    # Checking for valid input
    pubcheck(data)

    if armed and (data.conf_0 == 1 and data.conf_1 == 1): #Change conf statement to 'or'? That way control can still be done even if only one eye works
        # Conversion from linear input to exponential PWM output
        bluePWM = ((26.5 * (data.lin_x ** 3)) + 128)
        orangePWM = ((26.5 * ((-1 * data.ang_z) ** 3)) + 128)

        # Ensuring PWM does not exceed limits
        if bluePWM > MAXPWM:
            bluePWM = MAXPWM
        if orangePWM > MAXPWM:
            orangePWM = MAXPWM
        if bluePWM < MINPWM:
            bluePWM = MINPWM
        if orangePWM < MINPWM:
            orangePWM = MINPWM

    elif not armed:
        bluePWM = 128
        orangePWM = 128

    # Passing Through Lin X and Ang Z Values to cmd_vel from gaze_msg
    linXPassThrough = data.lin_x
    angZPassThrough = data.ang_z

    # Writing Serial values to Arduino
    # Message format is "XXXX YYYY" , but is intended as "XXX.x YYY.y"
    # the decimal point itself cannot be sent serially,
    # but the last value in each number is interpreted as a decimal by the arduino code
    # Current Message: BBBOOOs
    bluePWM = round(bluePWM)
    orangePWM = round(orangePWM)
    testString = str(bluePWM) + str(orangePWM) + "s"
    ser.write(testString.encode('utf-8'))

    # Publishing values to cmd_vel to control ROSBot
    rosBotMessage.linear.x = linXPassThrough
    rosBotMessage.angular.z = angZPassThrough
    pub_vel.publish(rosBotMessage)

    # Print values for debugging
    print(bluePWM, orangePWM)
    # print(data.ang_z)
    # print(testString)


if __name__ == '__main__':
    rospy.init_node('ArduinoInterpreter_ROSBot', anonymous=True)
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("/gaze_info", gaze_msg, callback)
    rospy.spin()
