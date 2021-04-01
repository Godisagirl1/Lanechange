#!/bin/bash
#Usage: ./GetResetTimeCosts.sh <type, possible values are 1 or 2>
awk '/Reset '$1'/ {print $9}'
