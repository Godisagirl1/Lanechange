#!/bin/bash
#Usage ./analyse_lanechanging_time_costs.sh <starting simulation ID (inclusive)> <ending simulation ID (exclusive)>
id=${1}
if [ -f sim${1}to${2}lanechanging_time_costs.m ]
then
  echo "rm sim${1}to${2}lanechanging_time_costs.m"
  rm sim${1}to${2}lanechanging_time_costs.m
fi
echo "touch sim${1}to${2}lanechanging_time_costs.m"
touch sim${1}to${2}lanechanging_time_costs.m

while [ $id -lt ${2} ]
do
  echo "grep 'Lanechange succeeded' output${id}.txt | awk '{print \$3}' >> sim${1}to${2}lanechanging_time_costs.m"
  grep 'Lanechange succeeded' output${id}.txt | awk '{print $3}' >> sim${1}to${2}lanechanging_time_costs.m
  ((id++))
done

echo "java -classpath bin AnalyseLanechanngingTimeCosts ${1} ${2} > report_lanechanging_time_costs.txt"
java -classpath bin AnalyseLanechangingTimeCosts ${1} ${2} > report_lanechanging_time_costs.txt
