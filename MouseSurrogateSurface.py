#!/usr/bin/env python3
from pynput import mouse
from falcongaze.msg import gaze_msg
import rospy
import time

# Creating a new gaze message for publishing to
gazeMessage = gaze_msg()


class mouseReader:

    # Tracks the current coords of the mouse
    currX = 0
    currZ = 0
    offScreenFlag = False
    ConfFlag = True

    def __init__(self):
        # Setting up the publisher and mouse listener
        rospy.init_node('glassesSimPublisher', anonymous=True)
        self.glassesSimPub = rospy.Publisher("/gaze_info", gaze_msg, queue_size=100)
        # rate = rospy.Rate(2)  # 10hz
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
        # print('{0} at {1}'.format(
        #    'Pressed' if pressed else 'Released',
        #    (x, y)))
        if pressed:
            self.ConfFlag = False
        else:
            self.ConfFlag = True



    # Puts a small box in the upper left corner that registers as "off screen" for testing
    def off_screen(self):
        if 0 <= self.currX <= 40 and 0 <= self.currZ <= 40:
            print("Off screen!")
            self.offScreenFlag = True
        else:
            print("On screen!")
            self.offScreenFlag = False

    def simPub(self):
        while not rospy.is_shutdown():
            self.off_screen()
            # Default Values
            if self.offScreenFlag:
                gazeMessage.surface = 0
            else:
                gazeMessage.surface = 1

            gazeMessage.mode = 1

            if self.ConfFlag:
                gazeMessage.conf_0 = 1  # right eye
                gazeMessage.conf_1 = 1  # left eye
            else:
                gazeMessage.conf_0 = 0  # right eye
                gazeMessage.conf_1 = 0  # left eye

            # Mapping x and y coordinates from mouse to -1 <-> 1 value
            gazeMessage.lin_x = ((-1 * (1.0 / 540.0) * self.currX) + 1)
            gazeMessage.ang_z = ((-1 * (1.0 / 960.0) * self.currZ) + 1)
            # print(gazeMessage)
            # Publishing gaze message
            self.glassesSimPub.publish(gazeMessage)
            rospy.sleep(0.0075)
            # sleep for 0.0075 for 125 hz


if __name__ == '__main__':
    testObj = mouseReader()
    testObj.simPub()  # !/usr/bin/env python
