#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/.bashrc
source /home/$USER/catkin_ws/devel/setup.bash

catkin build -DCMAKE_BUILD_TYPE=Release



