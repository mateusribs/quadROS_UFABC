#!/bin/bash

set -e
 
echo "########################### CONFIGURING ROS ENVIRONMENT #################################################"

source /opt/ros/noetic/setup.bash
catkin_init_workspace
cd ..
catkin_make
echo "source /home/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

pip install -r requirements.txt