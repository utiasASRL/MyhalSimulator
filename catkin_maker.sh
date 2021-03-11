#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/.bashrc
source /home/$USER/catkin_ws/devel/setup.bash

catkin build -DCMAKE_BUILD_TYPE=Release



