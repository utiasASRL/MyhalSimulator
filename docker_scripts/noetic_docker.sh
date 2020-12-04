#!/bin/bash

echo ""
echo "Running ros-noetic docker"
echo ""

if [ -n "$1" ]; then
  echo -e "Running command $1\n"
fi

rosport=$ROSPORT

# while getopts p: option
# do
# case "${option}"
# in
# p) rosport=${OPTARG};; 
# esac
# done

if [[ -z "$ROSPORT" ]]; then
    echo "WARNING: didn't provide ROSPORT, setting it to 1100"
    export ROSPORT=1100
fi

last_line=$(tail -1 ~/.bashrc)
s=${last_line:0:14}
if [[ "$s" == "export ROSPORT" ]]; then
    sed -i '$d' ~/.bashrc
fi

echo "ROSPORT=$rosport"
gazport=$(($rosport+1))
export ROSPORT=$(($ROSPORT+2))
echo "export ROSPORT=$ROSPORT" >> ~/.bashrc

docker run --gpus all -it --rm --shm-size=64g \
-v /raid/Myhal_Simulation:/home/$USER/Myhal_Simulation \
-v /raid/hth/Data/MyhalSim:/home/$USER/Data/MyhalSim \
-v /raid/Myhal_Simulation/Simulator/JackalNoetic:/home/$USER/catkin_ws \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/$USER/.Xauthority:/home/$USER/.Xauthority \
--net=host \
-e XAUTHORITY=/home/$USER/.Xauthority \
-e DISPLAY=$DISPLAY \
-e ROS_MASTER_URI=http://obelisk:$rosport \
-e GAZEBO_MASTER_URI=http://obelisk:$gazport \
-e ROSPORT=$rosport \
--name "$USER-noetic-$ROSPORT" \
docker_ros_noetic_$USER \
$1 

source ~/.bashrc
