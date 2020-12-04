#!/bin/bash

now=`date +%Y-%m-%d_%H-%M-%S`

nohup ./multi_tours.sh -n > "/raid/Myhal_Simulation/simulated_runs/nohup_log_$now.txt" 2>&1 &

echo "Started nohup process. PID:"
echo $!
echo ""
