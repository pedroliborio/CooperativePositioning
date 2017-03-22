#!/bin/bash

for seed in {0..29}; #trafego CBR
do
	taskset -c 2 ../CooperativePositioning -u Cmdenv -r $seed -n ..:../../veins/examples/veins:../../veins/src/veins --image-path=../../veins/images -l ../../veins/src/veins --debug-on-errors=false omnetpp_rclt.ini > saida_$seed_rclt.txt
done