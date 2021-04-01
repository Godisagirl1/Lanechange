#!/bin/bash
#Usage: ./extract_time_headway.sh <number of simulations> <number of targetlane vehicles> <sampling step size, unit logs/sample>
java -classpath bin ExtractTimeHeadway $1 $2 $3

if [ -f sim0to$1_time_headways.m ]
then
  rm sim0to$1_time_headways.m
fi
touch sim0to$1_time_headways.m

simId=0
while [ $simId -lt $1 ]
do
  cat sim${simId}_time_headways.m >> sim0to$1_time_headways.m
  ((simId++))
done
