#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "usage $0 <CSV file> <SQL file>"
    exit
fi

for l in `cat $1`; do echo $l | awk '{split($0,a,","); print "insert into run (input,size,exit,runtime,memuse) values(\x27" a[4] "\x27," a[1] ",0," a[2] "," a[3] ");"}' | sqlite3 $2; done
