#!/bin/bash
echo Terminal 1
source /opt/ros/noetic/setup.bash
roscore
rosrun falcongaze MouseSurrogateSurface.py
echo Launch Successful
