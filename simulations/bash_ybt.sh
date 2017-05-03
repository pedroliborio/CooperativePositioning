#!/bin/bash

for seed in {0..14}; #trafego CBR
do
	taskset -c 14 ../CooperativePositioning -u Cmdenv -r $seed -c YBT -n ..:../../veins/examples/veins:../../veins/src/veins --image-path=../../veins/images -l ../../veins/src/veins --debug-on-errors=false omnetpp.ini
done