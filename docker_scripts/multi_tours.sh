#!/bin/bash

################
# nohup option #
################

nohup=false

while getopts n option
do
case "${option}"
in
n) nohup=true ;;
esac
done

nohup_arg=""
if [ "$nohup" = true ] ; then
  nohup_arg="-n"
fi

#####################
# Common parameters #
#####################

#MAPPING=0
#ARGS="-f"

##############
# Loop Tours #
##############


for MAPPING in "2"
do
  for ARGS in "-fg"
  do
    #for PARAMS in "Flow1_params" "Flow2_params" "Flow3_params"
    for PARAMS in "Flow2_params" "Flow3_params"
    do
        for TOUR in "A_tour" "B_tour" "C_tour"
        #for TOUR in  "A_short"
        do
            echo ""
            echo "|--------------------------------------------------------------------------"
            echo "|"
            echo "|    Starting new experiment with $PARAMS on $TOUR"
            echo "|    *************************************************"
            echo "|"
            echo "|"
            echo ""
            if [ "$ARGS" = "-fe"] ; then
              ./classification_test.sh -c "./master.sh $ARGS -m $MAPPING -t $TOUR -p $PARAMS"
            else
              ./melodic_docker.sh $nohup_arg -c "./master.sh $ARGS -m $MAPPING -t $TOUR -p $PARAMS"
            fi
            sleep 5
        done
    done
  done
done