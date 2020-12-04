#!/bin/bash

########
# Init #
########

echo ""
echo "Running ros-melodic and ros-noetic docker. Remember you can set ROSPORT to a custom value"
echo ""

rosport=$ROSPORT
detach=false
command=""

while getopts dc: option
do
case "${option}"
in
d) detach=true;;
c) command=${OPTARG};;
esac
done

if [ -n "$command" ]; then
  echo -e "Running command $command\n"
fi

if [[ -z "$ROSPORT" ]]; then
    echo "WARNING: didn't provide ROSPORT, setting it to random value, this could result in conflicts." 1>&2
    export ROSPORT=$(($RANDOM%30000+1101))
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

###########################
# Start docker containers #
###########################

# Docker run arguments (depending if we run detached or not)
if [ "$nohup" = true ] ; then
    docker_args="-i --rm --shm-size=64g --gpus all " 
else
    docker_args="-it --rm --shm-size=64g --gpus all "
fi

# Running detached
if [ "$detach" = true ] ; then
    docker_args="-d ${docker_args}"

# Create folder for simulation if not already there
mkdir -p "$PWD/../../Simulation_Data/simulated_runs"

# Volumes (modify with your own path here)
volumes_melodic="-v $PWD/..:/home/$USER/catkin_ws \
-v $PWD/../../Simulation_Data:/home/$USER/Myhal_Simulation"

volumes_noetic="-v $PWD/../../MyhalSimulator-DeepPreds:/home/$USER/catkin_ws \
-v $PWD/../../Simulation_Data:/home/$USER/Myhal_Simulation \
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
docker run -d --gpus all -i --rm --shm-size=64g \
$volumes_melodic \
$other_args \
--name "$USER-melodic-$ROSPORT" \
docker_ros_melodic_$USER \
$command &&
docker run $docker_args \
$volumes_noetic \
$other_args \
--name "$USER-noetic-$ROSPORT" \
docker_ros_noetic_$USER \
"./classifier.sh"