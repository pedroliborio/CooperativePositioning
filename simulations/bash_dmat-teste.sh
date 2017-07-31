#!/bin/bash

for seed in {0..30}; #trafego CBR
do
	../CooperativePositioning -u Cmdenv -c DMAT -n ..:../../veins/examples/veins:../../veins/src/veins --image-path=../../veins/images -l ../../veins/src/veins --debug-on-errors=false omnetpp.ini --num-rngs=10 --repeat=1 --seed-set=$seed
done
