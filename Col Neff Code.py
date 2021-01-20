#!/usr/bin/env python3

import numpy as np
import cv2
import imutils
from time import time
import rospy
from geometry_msgs.msg import Twist
from falcongaze.msg import gaze_msg, conf_msg

coords = (0,0)
lb = False
rb = False
timeLD = 0.0
timeLU = 0.0
timeRD = 0.0
timeRU = 0.0

#Global color declaration
white = (255,255,255)
black = (0,0,0)
blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)

# Create publishers/subscribers
pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
output = Twist()

class mouseData(object):
    def __init__(self,x=0,y=0,lb=False,rb=False):
        self.x=x
        self.y=y
        self.lb=lb
        self.rb=rb
        self.time_LD = 0.0
        self.time_LU = 0.0
        self.time_RD = 0.0
        self.time_RU = 0.0
        self.armed = False
        
    def setPos(self,coords):
        self.x = coords[0]
        self.y = coords[1]

    def setBut(self,lb,rb):
        self.lb = lb
        self.rb = rb

    def printCirc(self,frame):
        cv2.circle(frame,(self.x,self.y),10,green,-1)
        cv2.imshow('Falcon Gaze Controller',frame)

    def LTime(self,timeLD,timeLU):
        self.time_LU = timeLU
        self.time_LD = timeLD
        clickL = self.time_LU - self.time_LD
        if clickL > 3.0 and self.armed == False:
            self.armed = True
            print("Left click length is %d"%(clickL))


    def RTime(self,timeRD,timeRU):
        self.time_RU = timeRU
        self.time_RD = timeRD
        clickL = self.time_RU - self.time_RD
        if clickL > 3.0 and self.armed == True:
            self.armed = False
            resetClick()
            print("Right click length is %d"%(clickL))


def editFrame(frame,mouseD):
    pts = np.array([[300, 150], [400, 50], [500, 150]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(frame, [pts], True, red,5)
    pts = np.array([[300, 450], [400, 550], [500, 450]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(frame, [pts], True, red, 5)
    pts = np.array([[200, 200], [100, 300], [200, 400]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(frame, [pts], True, red, 5)
    pts = np.array([[600, 200], [700, 300], [600, 400]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(frame, [pts], True, red, 5)
    if mouseD.armed == False:
#         print(str(mouseD.armed))
        frame = cv2.putText(frame,'Disarmed',(375,275), cv2.FONT_HERSHEY_SIMPLEX, 1.0,red,lineType=cv2.LINE_AA)
    else:
        frame = cv2.putText(frame,'Armed',(375,275), cv2.FONT_HERSHEY_SIMPLEX, 1.0,green,lineType=cv2.LINE_AA)
    return frame

def resetClick():
    global timeLD
    global timeLU
    global timeRD
    global timeRU
    timeLD = 0.0
    timeLU = 0.0
    timeRD = 0.0
    timeRU = 0.0

def onMouse(event, x, y, flags, param):
    global coords
    global lb
    global rb
    global timeLD
    global timeLU
    global timeRD
    global timeRU

    if event == cv2.EVENT_MOUSEMOVE:
        coords = (x,y)
    if event == cv2.EVENT_LBUTTONDOWN:
        lb = True
        timeLD = time()
    if event == cv2.EVENT_LBUTTONUP:
        lb = False
        timeLU = time()
    if event == cv2.EVENT_RBUTTONDOWN:
        rb = True
        timeRD = time()
    if event == cv2.EVENT_RBUTTONUP:
        rb = False
        timeRU = time()
        
def deadZone():
    global output
    if output.linear.x < .3 and output.linear.x > -.3:
        output.linear.x = 0
    if output.angular.z < .3 and output.angular.z > -.3:
        output.angular.z = 0
    output.linear.x = output.linear.x*output.linear.x
    output.angular.z = output.angular.z*output.angular.z

def pubData(mouseD):
    if mouseD.armed == True:
        output.linear.x = -1*(mouseD.y-300)/300
        output.angular.z = -1*(mouseD.x-400)/400
    else:
        output.linear.x = 0
        output.angular.z = 0
    deadZone()
    
    pub_vel.publish(output)
    

def main():
    global output
    mousey = mouseData()

    cap = cv2.VideoCapture(0)
    cap.set(5,30)

    white = (255,255,255)
    black = (0,0,0)
    blue = (255,0,0)
    green = (0,255,0)
    red = (0,0,255)
    
    rospy.loginfo("Starting Gaze Info process.")
    print("Gaze Info process running.")
    print("Hold the left mouse button down for 3 seconds to arm")
    
    rospy.init_node('pupil_gaze')
    rate = rospy.Rate(125) 
    
    while not rospy.is_shutdown():
    # Capture frame-by-frame
        ret, frame = cap.read()
        frame = imutils.resize(frame, width=800)
        height,width = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = editFrame(frame,mousey)

        # Display the resulting frame
        cv2.namedWindow('Falcon Gaze Controller')
        cv2.setMouseCallback('Falcon Gaze Controller', onMouse)
        mousey.setPos(coords)
        mousey.setBut(lb,rb)
        # print("(%d,%d)"%(mousey.x,mousey.y) + str(lb) + "," + str(rb))
        mousey.printCirc(frame)
        mousey.LTime(timeLD,timeLU)
        mousey.RTime(timeRD,timeRU)
        print(str(mousey.armed))
        pubData(mousey)

        key = cv2.waitKey(1) &0xFF
        if key == 27 or key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break



       


# When everything done, release the capture
if __name__=="__main__":
    main()
