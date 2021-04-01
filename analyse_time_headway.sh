#!/bin/bash
#Usage ./analyse_lanechanging_time_costs.sh <starting simulation ID (inclusive)> <ending simulation ID (exclusive)>

echo "java -classpath bin AnalyseTimeHeadway ${1} ${2} > report_time_headway.txt"
java -classpath bin AnalyseTimeHeadway ${1} ${2} > report_time_headway.txt
