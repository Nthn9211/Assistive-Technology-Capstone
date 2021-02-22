#!/bin/bash
echo Terminal 2
source /opt/ros/noetic/setup.bash
source ~/gaze_ws/devel/setup.bash
rosrun falcongaze ArduinoInterpreter.py
echo Execution complete, shutting down now
shutdown now
