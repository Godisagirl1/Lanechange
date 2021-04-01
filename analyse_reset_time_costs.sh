#!/bin/bash
#Usage ./analyse_lanechanging_time_costs.sh <starting simulation ID (inclusive)> <ending simulation ID (exclusive)>

echo "java -classpath bin AnalyseResetTimeCosts ${1} ${2} > report_reset_time_costs.txt"
java -classpath bin AnalyseResetTimeCosts ${1} ${2} > report_reset_time_costs.txt
