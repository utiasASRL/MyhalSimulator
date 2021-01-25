#!/bin/bash

########
# Init #
########

echo ""
echo "Running ros-collider docker"
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

##########################
# Start docker container #
##########################

# Docker run arguments
docker_args="-it --rm --shm-size=64g "

# Running on gpu (Uncomment to enable gpu)
docker_args="${docker_args} --gpus all "

# Create folder for simulation if not already there
mkdir -p "$PWD/../../Simulation_Data/simulated_runs"

# Volumes (modify with your own path here)
volumes="-v $PWD/../../MyhalSimulator-DeepCollider:/home/$USER/catkin_ws \
-v $PWD/../../Simulation_Data:/home/$USER/Myhal_Simulation \
-v $PWD/../../MyhalSimulator:/home/$USER/MyhalSimulator \
-v $PWD/../../KPConv_Data:/home/$USER/Data/MyhalSim"

# Additional arguments to be able to open GUI
XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
other_args="-v $XSOCK:$XSOCK \
    -v $XAUTH:$XAUTH \
    --net=host \
    --privileged \
	-e XAUTHORITY=${XAUTH} \
    -e DISPLAY=$DISPLAY \
    -e ROS_MASTER_URI=http://$HOSTNAME:$rosport \
    -e GAZEBO_MASTER_URI=http://$HOSTNAME:$gazport \
    -e ROSPORT=$rosport "


# Execute the command in docker (Example of command: ./master.sh -ve -m 2 -p Sc1_params -t A_tour)
docker run $docker_args \
$volumes \
$other_args \
--name "$USER-training-$ROSPORT" \
docker_ros_collider_$USER \
$1

source ~/.bashrc

