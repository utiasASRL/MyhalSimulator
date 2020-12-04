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
  for ARGS in "-e" "-fge" "-fe"
  do
    for PARAMS in "Sc1_params" "Sc2_params" "Sc3_params" "Sc4_params"
    do
        #for TOUR in "A_tour" "B_tour" "C_tour"
        for TOUR in  "A_short"
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