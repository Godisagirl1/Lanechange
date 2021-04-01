#!/bin/bash
id="${1}"
#awk 'BEGIN{ print "tar"'"$id"'" = [" } \
#/^T\[Tar '"$id"' / {print $4, $7, $8} \
#END{ print "];\n" }'
#$2 = id, $4 = time (tick), $17 = X, $20 = Y, $23 = psi, $26 = x, $29 = y, $32 =dot_X, $35 = dot_Y, $38 = dot_psi,$41, $44,
awk '/^T\[Tar '"$id"' / {print $4, $17, $20, $32, $35}'
