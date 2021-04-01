#!/bin/bash

awk '/starts waiting for reset/ || /Reset/ {print $1, $3, $5, $6, $9}'
