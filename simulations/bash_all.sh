#!/bin/bash

for seed in {0..14}; #trafego CBR
do
	taskset -c 1 ../CooperativePositioning -u Cmdenv -r $seed -c DMAT -n ..:../../veins/examples/veins:../../veins/src/veins --image-path=../../veins/images -l ../../veins/src/veins --debug-on-errors=false omnetpp.ini
done

for seed in {0..14}; #trafego CBR
do
	taskset -c 2 ../CooperativePositioning -u Cmdenv -r $seed -c DPT -n ..:../../veins/examples/veins:../../veins/src/veins --image-path=../../veins/images -l ../../veins/src/veins --debug-on-errors=false omnetpp.ini
done

for seed in {0..14}; #trafego CBR
do
	taskset -c 3 ../CooperativePositioning -u Cmdenv -r $seed -c RCLT -n ..:../../veins/examples/veins:../../veins/src/veins --image-path=../../veins/images -l ../../veins/src/veins --debug-on-errors=false omnetpp.ini
done

for seed in {0..14}; #trafego CBR
do
	taskset -c 4 ../CooperativePositioning -u Cmdenv -c RIO -r $seed -n ..:../../veins/examples/veins:../../veins/src/veins --image-path=../../veins/images -l ../../veins/src/veins --debug-on-errors=false omnetpp.ini
done

for seed in {0..14}; #trafego CBR
do
	taskset -c 5 ../CooperativePositioning -u Cmdenv -r $seed -c YBT -n ..:../../veins/examples/veins:../../veins/src/veins --image-path=../../veins/images -l ../../veins/src/veins --debug-on-errors=false omnetpp.ini
done
