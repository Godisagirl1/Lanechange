#!/bin/bash
#awk 'BEGIN{ print "r = [" } \
#/^T\[R/ {print $3, $6, $7} \
#END{ print "];\n" }'
#$3 = time (tick), $7 = node, $13 = DrivingMode, $16 = X, $19 = Y, $22 = psi, $25 = x, $28 = y, $31, $34, $37, $40, $43, $46, $49, $52, $55, $58
awk '/^T\[R/ {print $3, $16, $19, $31, $34}'
