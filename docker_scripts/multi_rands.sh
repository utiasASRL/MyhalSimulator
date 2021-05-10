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
  nohup_arg="-d"
fi

#####################
# Common parameters #
#####################

#MAPPING=0
#ARGS="-f"

##############
# Loop Tours #
##############

for i in {1..7}
do
  for MAPPING in "2"
  do
    for ARGS in "-fg"
    do
      # for PARAMS in "RandBounce_params"
      # for PARAMS in "RandWand_params"
      for PARAMS in "RandFlow_params"
      do
        for TOUR in "A_tour" "B_tour" "C_tour"
        #for TOUR in "A_short"
        do

          echo ""
          echo "|--------------------------------------------------------------------------"
          echo "|"
          echo "|    Starting new experiment with $PARAMS on $TOUR"
          echo "|    *************************************************"
          echo "|"
          echo "|"
          echo ""

          sleep 1

          # Wait until last docker container has finished
          ROSPORT_USED=$(($ROSPORT + 2))
          echo "Waiting for container hth-melodic-$ROSPORT_USED to be finished"
          while [ "$(docker ps -q -f name=hth-melodic-$ROSPORT_USED)" ]
          do 
            sleep 1
          done 
          echo "OK"

          # Clean up container
          if [ "$(docker ps -aq -f status=exited -f name=hth-melodic-$ROSPORT_USED)" ]; then
              docker rm hth-melodic-$ROSPORT_USED
          fi

          # Start new one
          if [ "$ARGS" == "-fe" ] ; then
            ./classification_test.sh -c "./master.sh $ARGS -m $MAPPING -t $TOUR -p $PARAMS"
          else
            ./melodic_docker.sh $nohup_arg -c "./master.sh $ARGS -m $MAPPING -t $TOUR -p $PARAMS"
          fi

          sleep 1

        done
      done
    done
  done
done

