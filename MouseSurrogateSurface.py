#!/usr/bin/env python3
from pynput import mouse
from falcongaze.msg import gaze_msg
import rospy
import time

# Creating a new gaze message for publishing to with custom gaze format
gazeMessage = gaze_msg()

class mouseReader:

    # Tracks the current coords of the mouse
    currX = 0
    currZ = 0
    offScreenFlag = False
    ConfFlag = True #Confidence high (true) or low (false)

    def __init__(self):
        # Setting up the publisher and mouse listener
        rospy.init_node('glassesSimPublisher', anonymous=True)
        self.glassesSimPub = rospy.Publisher("/gaze_info", gaze_msg, queue_size=100)
        listenerMouse = mouse.Listener(
            on_move=self.on_move,
            on_click=self.on_click)
        listenerMouse.start()

    def on_move(self, x, y):
        # Get position of mouse
        self.currX = y
        self.currZ = x

    def on_click(self, x, y, button, pressed):
        # Recognize mouse clicks
        if pressed:
            self.ConfFlag = False
        else:
            self.ConfFlag = True



    # Puts a small box in the upper left corner that registers as "off screen" for testing
    def off_screen(self):
        if 0 <= self.currX <= 40 and 0 <= self.currZ <= 40:
            # print("Off screen!")
            self.offScreenFlag = True
        else:
            # print("On screen!")
            self.offScreenFlag = False

    def simPub(self): #Main function
        while not rospy.is_shutdown(): #While running ROS, run main
        
            self.off_screen()#Checking if we are in the small box in corner (surface on/off)
            
            # Default Values
            if self.offScreenFlag: # If we are in the box, set surface to 0, else 1
                gazeMessage.surface = 0 
            else:
                gazeMessage.surface = 1

            gazeMessage.mode = 1 #Default, unused value

            if self.ConfFlag: #If confidence high, publish 1 for both eyes, else 0
                gazeMessage.conf_0 = 1  # right eye
                gazeMessage.conf_1 = 1  # left eye
            else:
                gazeMessage.conf_0 = 0  # right eye
                gazeMessage.conf_1 = 1  # left eye

            # Mapping x and y coordinates from mouse to a -1 <-> 1 value based on screen resolution
            gazeMessage.lin_x = ((-1 * (1.0 / 540.0) * self.currX) + 1) #Function that changes mouse coordinates to a value between -1 and 1 that simulates eye gaze on surface
            gazeMessage.ang_z = ((-1 * (1.0 / 960.0) * self.currZ) + 1)
            print(gazeMessage)
            # Publishing gaze message
            self.glassesSimPub.publish(gazeMessage)
            rospy.sleep(0.0075)
            # sleep for 0.0075 sec for 125 hz publish rate to match real glasses publish rate


if __name__ == '__main__':
    testObj = mouseReader()
    testObj.simPub()  # !/usr/bin/env python
