#!/usr/bin/env python
from pynput import mouse
from wheelchair.msg import gaze_msg
import rospy
import time

#Creating a new gaze message for publishing to
gazeMessage = gaze_msg()

class mouseReader:

    #Tracks the current coords of the mouse
    currX = 0
    currZ = 0

    def __init__(self):
        #Setting up the publisher and mouse listener
        rospy.init_node('glassesSimPublisher', anonymous=True)
        self.glassesSimPub = rospy.Publisher("simGlassValues", gaze_msg, queue_size=10)
        rate = rospy.Rate(10)  # 10hz
        listener = mouse.Listener(
            on_move=self.on_move,
            on_click=self.on_click)
        listener.start()

    def on_move(self,x, y):
        #Get position of mouse
        self.currX = y
        self.currZ = x

    def on_click(self,x, y, button, pressed):
        #Recognize mouse clicks
        print('{0} at {1}'.format(
            'Pressed' if pressed else 'Released',
            (x, y)))

    def simPub(self):
        while(not rospy.is_shutdown()):
            #Default Values
            gazeMessage.surface = 1
            gazeMessage.mode = 1
            gazeMessage.conf_0 = 1
            gazeMessage.conf_1 = 1
            #Mapping x and y coordinates from mouse to -1 <-> 1 value
            gazeMessage.lin_x = ((-1*(1.0/540.0)*self.currX)+1)
            gazeMessage.ang_z = ((-1*(1.0/960.0)*self.currZ)+1)
            print(gazeMessage)
            #Publishing gaze message
            self.glassesSimPub.publish(gazeMessage)

if __name__ == '__main__':
    testObj = mouseReader()
    testObj.simPub()