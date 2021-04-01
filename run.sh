#!/bin/bash
#Usage: ./run.sh <starting simulation ID (inclusive)> <ending simulation ID (exclusive)> <starting ID of targetlane vehicle (inclusive)> <ending ID of targetlane vehicle (exclusive)>
counter=$1

if [ -f sim$1to$2reset1_time_costs.m ]
then
  rm sim$1to$2reset1_time_costs.m
fi
touch sim$1to$2reset1_time_costs.m

if [ -f sim$1to$2reset2_time_costs.m ]
then
  rm sim$1to$2reset2_time_costs.m
fi
touch sim$1to$2reset2_time_costs.m

if [ -f sim$1to$2reset_time_costs.m ]
then
  rm sim$1to$2reset_time_costs.m
fi
touch sim$1to$2reset_time_costs.m

while [ $counter -lt $2 ]
do
  echo "./sim.sh > output$counter.txt"
  ./sim.sh > output$counter.txt
  echo "extracting data:"
  id=$3
  while [ $id -lt $4 ]
  do
    echo "cat output${counter}.txt | ./GetTarTrajectory.sh $id > sim${counter}tar$id.m"
    cat output${counter}.txt | ./GetTarTrajectory.sh $id > sim${counter}tar$id.m
    ((id++))
  done
  echo "cat output${counter}.txt | ./GetRTrajectory.sh > sim${counter}r.m"
  cat output${counter}.txt | ./GetRTrajectory.sh > sim${counter}r.m
  echo "cat output${counter}.txt | ./GetResetTrajectoryForValidation.sh > sim${counter}reset_validation.m"
  cat output${counter}.txt | ./GetResetTrajectoryForValidation.sh > sim${counter}reset_validation.m
  echo "cat output${counter}.txt | ./GetResetTimeCosts.sh 1 >> sim$1to$2reset1_time_costs.m"
  cat output${counter}.txt | ./GetResetTimeCosts.sh 1 >> sim$1to$2reset1_time_costs.m
  echo "cat output${counter}.txt | ./GetResetTimeCosts.sh 2 >> sim$1to$2reset2_time_costs.m"
  cat output${counter}.txt | ./GetResetTimeCosts.sh 2 >> sim$1to$2reset2_time_costs.m
  echo "data extracted."
  ((counter++))
done

echo "cat sim$1to$2reset1_time_costs.m > sim$1to$2reset_time_costs.m"
cat sim$1to$2reset1_time_costs.m > sim$1to$2reset_time_costs.m
echo "cat sim$1to$2reset2_time_costs.m >> sim$1to$2reset_time_costs.m"
cat sim$1to$2reset2_time_costs.m >> sim$1to$2reset_time_costs.m

