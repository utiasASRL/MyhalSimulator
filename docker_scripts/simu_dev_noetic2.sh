#!/bin/bash

########
# Init #
########

echo ""
echo "Running ros-noetic2 docker. Remember you can set ROSPORT to a custom value"
echo ""

rosport=$ROSPORT
detach=false
nohup=false
command=""

while getopts dnc: option
do
case "${option}"
in
d) detach=true;;
n) nohup=true;;
c) command=${OPTARG};;
esac
done

if [ -n "$command" ]; then
  echo -e "Running command $command\n"
fi

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

# Docker run arguments (depending if we run detached or not)
if [ "$detach" = true ] ; then
    docker_args="-d -it --rm --shm-size=64g "
else
    if [ "$nohup" = true ] ; then
        docker_args="-i --rm --shm-size=64g " 
    else
        docker_args="-it --rm --shm-size=64g "
    fi 
fi

# Running on gpu (Uncomment to enable gpu)
docker_args="${docker_args} --gpus all "

# Create folder for simulation if not already there
mkdir -p "$PWD/../../Simulation_Data/simulated_runs"

# Volumes (modify with your own path here)
volumes="-v $PWD/..:/home/$USER/catkin_ws \
-v $PWD/../../Simulation_Data:/home/$USER/Myhal_Simulation "

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
    -e ROSPORT=$rosport \
    --env GAZEBO_MODEL_PATH=/home/$USER/catkin_ws/src/myhal_simulator/models \
    --env GAZEBO_RESOURCE_PATH=/home/$USER/catkin_ws/src/myhal_simulator/models \
    --env GAZEBO_PLUGIN_PATH=/home/$USER/catkin_ws/devel/lib"


# Execute the command in docker (Example of command: ./master.sh -ve -m 2 -p Sc1_params -t A_tour)
docker run $docker_args \
$volumes \
$other_args \
--name "$USER-simu2-dev" \
docker_ros_noetic2_$USER \
$command 

# Finish
echo "Final Sourcing ..."
source ~/.bashrc


