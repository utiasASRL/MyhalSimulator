#!/bin/bash

#######
# Intro
#######

# First some sources and imports
source /home/$USER/.bashrc
source devel/setup.bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/devel/lib

# Get the commad that started this file
myInvocation="$(printf %q "$BASH_SOURCE")$((($#)) && printf ' %q' "$@")"

#################
# Parse arguments
#################

GUI=false # -v flag
TOUR="A_tour" # -t (arg) flag
LOADWORLD="" # -l (arg) flag
FILTER=false # -f flag
MAPPING=2 # -m (arg) flag
t=$(date +'%Y-%m-%d-%H-%M-%S')
GTCLASS=false # -g flag 
VIZ_GAZ=false
PARAMS="default_params"

while getopts p:t:l:m:n:vfge option
do
case "${option}"
in
p) PARAMS=${OPTARG};; # what param file are we using?
t) TOUR=${OPTARG};; # What tour is being used 
l) LOADWORLD=${OPTARG};; # do you want to load a prexisting world or generate a new one
m) MAPPING=${OPTARG};; # use gmapping, AMCL or PointSLAM? (respectively 0, 1, 2)
n) t=${OPTARG};; # Overwrite the date
v) GUI=true;; # using gui?
f) FILTER=true;; # pointcloud filtering?
g) GTCLASS=true;; # are we using ground truth classifications, or online_classifications
e) VIZ_GAZ=true;; # are we going to vizualize topics in gazebo
esac
done


#########################################################################################################################
echo ""
echo ""
echo "    +-----------------+"
echo "    | Command summary |"
echo "    +-----------------+"
echo ""

echo "Folder Name: $t"

MINSTEP=0.0001
echo "Min step size: $MINSTEP"

echo -e "TOUR: $TOUR\nGUI: $GUI\nLOADWORLD: $LOADWORLD\nFILTER: $FILTER\nMAPPING: $MAPPING\nGTCLASS: $GTCLASS"

sleep 1

c_method="ground_truth"

if [ "$FILTER" = false ] ; then
    c_method="none"
    GTCLASS=false
else
    if [ "$GTCLASS" = false ] ; then
        c_method="online_predictions"
    else 
        c_method="ground_truth"
    fi
fi
export GTCLASSIFY=$GTCLASS

echo "Running tour: $TOUR"

#########################################################################################################################
echo ""
echo ""
echo "    +------------------+"
echo "    | Starting roscore |"
echo "    +------------------+"
echo ""

# Kill possible running ros nodes
killall gzserver
killall gzclient
killall rviz
killall roscore
killall rosmaster

# Start roscore with specific port
roscore -p $ROSPORT&

# Wait until rosmaster has started 
until rostopic list; do sleep 0.5; done 

echo ""
echo "roscore has started"
echo ""

# Load all the parameters in ros master
rosparam load src/myhal_simulator/params/$PARAMS/custom_simulation_params.yaml
rosparam load src/myhal_simulator/params/$PARAMS/common_vehicle_params.yaml
rosparam load src/myhal_simulator/params/$PARAMS/animation_params.yaml
rosparam load src/myhal_simulator/params/$PARAMS/room_params_V2.yaml
rosparam load src/myhal_simulator/params/$PARAMS/scenario_params_V2.yaml
rosparam load src/myhal_simulator/params/$PARAMS/plugin_params.yaml
rosparam load src/myhal_simulator/params/$PARAMS/model_params.yaml
rosparam load src/myhal_simulator/params/$PARAMS/camera_params.yaml
rosparam load src/myhal_simulator/tours/$TOUR/config.yaml
rosparam set gt_class $GTCLASS
rosparam set localization_test false
rosparam set class_method $c_method
rosparam set use_sim_time true
rosparam set tour_name $TOUR
rosparam set start_time $t
rosparam set filter_status $FILTER
rosparam set gmapping_status true
rosparam set loc_method $MAPPING
rosparam set min_step $MINSTEP
rosparam set viz_gaz $VIZ_GAZ

# Save parameters in log files
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t"
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t"
mkdir "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/videos/"
LOGFILE="/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/log.txt"
PARAMFILE="/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/params.yaml"
PCLFILE="/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/pcl.txt"
touch $LOGFILE
echo -e "Command used: $myInvocation" >> $LOGFILE
echo -e "\nPointcloud filter params: \n" >> $LOGFILE
echo -e "TOUR: $TOUR\nGUI: $GUI\nLOADWORLD: $LOADWORLD\nFILTER: $FILTER\nMAPPING: $MAPPING\nGTCLASS: $GTCLASS"  >> $LOGFILE
echo -e "$(cat /home/$USER/catkin_ws/src/jackal_velodyne/launch/include/pointcloud_filter2.launch)" >> $PCLFILE
echo -e "$(cat /home/$USER/catkin_ws/src/myhal_simulator/params/$PARAMS/room_params_V2.yaml)" > $PARAMFILE
echo -e "\n" >> $PARAMFILE
echo -e "$(cat /home/$USER/catkin_ws/src/myhal_simulator/params/$PARAMS/scenario_params_V2.yaml)" >> $PARAMFILE
echo -e "\n" >> $PARAMFILE
echo -e "$(cat /home/$USER/catkin_ws/src/myhal_simulator/params/$PARAMS/plugin_params.yaml)" >> $PARAMFILE
echo -e "\n" >> $PARAMFILE
echo -e "tour_name: $TOUR" >> $PARAMFILE

echo ""
echo "Parameters loaded"
echo ""

sleep 0.1

#########################################################################################################################
echo ""
echo ""
echo "    +-----------------------+"
echo "    | Creation of the world |"
echo "    +-----------------------+"
echo ""

WORLDFILE="/home/$USER/catkin_ws/src/myhal_simulator/worlds/myhal_sim.world"

if [[ -z $LOADWORLD ]]; then

    echo ""
    echo "Creating a world file"
    echo ""

    rosrun myhal_simulator world_factory
    rosparam set load_world "none"

else

    WORLDFILE="/home/$USER/Myhal_Simulation/simulated_runs/$LOADWORLD/logs-$LOADWORLD/myhal_sim.world"

    echo ""
    echo "Loading world $WORLDFILE"
    echo ""

    rosparam set load_world $LOADWORLD

fi

echo ""
echo "OK"
echo ""

cp $WORLDFILE "/home/$USER/Myhal_Simulation/simulated_runs/$t/logs-$t/"

sleep 0.1


#########################################################################################################################
echo ""
echo ""
echo "    +---------------------+"
echo "    | Starting simulation |"
echo "    +---------------------+"
echo ""

# Recording simulation data
rosbag record -O "/home/$USER/Myhal_Simulation/simulated_runs/$t/raw_data.bag" /clock /shutdown_signal /velodyne_points /move_base/local_costmap/costmap /move_base/global_costmap/costmap /ground_truth/state /map /move_base/NavfnROS/plan /amcl_pose /tf /tf_static /move_base/result /tour_data /optimal_path /classified_points &

# Start the python assessor
rosrun dashboard assessor.py &
echo -e "\033[1;4;34mRUNNING SIM\033[0m"
sleep 2.5


# Launch p1
roslaunch jackal_velodyne p1.launch gui:=$GUI world_name:=$WORLDFILE #extra_gazebo_args:="-s libdirector.so"

# In the end of the simu, run data processing
sleep 0.5
echo "Running data_processing.py"
rosrun dashboard data_processing.py $t $FILTER
 
exit 1



