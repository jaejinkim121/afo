#!/usr/bin/env bash
function cleanup { 
  echo "Cleaning up before exit..." 
  # Perform cleanup actions here 
  exit 1
} 
trap cleanup SIGINT
source /opt/ros/melodic/setup.bash
source /home/afo/catkin_ws/devel/setup.bash

roslaunch afo_launch afo.launch

