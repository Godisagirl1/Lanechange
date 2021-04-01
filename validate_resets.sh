#!/bin/bash
#Usage: ./validate_resets.sh <starting simulation ID (inclusive)> <ending simulation ID (exclusive)>
java -classpath bin ValidateResets $1 $2
