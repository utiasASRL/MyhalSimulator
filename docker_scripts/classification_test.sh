#!/bin/bash

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

volumes_melodic="-v /home/$USER/Experiments/2-MyhalSim/MyhalSimulator:/home/$USER/catkin_ws \
-v /home/$USER/Experiments/2-MyhalSim/Simulation:/home/$USER/Myhal_Simulation"

volumes_noetic="-v /home/$USER/Experiments/2-MyhalSim/MyhalSimulator-DeepPreds:/home/$USER/catkin_ws \
-v /home/$USER/Experiments/2-MyhalSim/Simulation:/home/$USER/Myhal_Simulation \
-v /home/$USER/Experiments/2-MyhalSim/KPConv-MyhalSimData:/home/$USER/Data/MyhalSim "

other_args="-v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/$USER/.Xauthority:/home/$USER/.Xauthority \
    --net=host \
    -e XAUTHORITY=/home/$USER/.Xauthority \
    -e DISPLAY=$DISPLAY \
    -e ROS_MASTER_URI=http://obelisk:$rosport \
    -e GAZEBO_MASTER_URI=http://obelisk:$gazport \
    -e ROSPORT=$rosport "


docker run -d --gpus all -i --rm --shm-size=64g \
$volumes_noetic \
$other_args \
--name "$USER-noetic-$ROSPORT" \
docker_ros_noetic_$USER \
"./classifier.sh" &&
docker run --gpus all -i --rm --shm-size=64g \
$volumes_melodic \
$other_args \
--name "$USER-melodic-$ROSPORT" \
docker_ros_melodic_$USER \
$command



