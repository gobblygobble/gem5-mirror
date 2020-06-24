#!/bin/bash

#for size in 128 256 512 1024
for size in 256 512 1024
do
    #for assoc in 2 4 8
    for assoc in 4 8
    do
        build/X86/gem5.opt configs/SPEC2006/speccpu2006-part1.py --benchmark=bzip2 --cpu-type=MinorCPU --caches --l2dtlb_size=$size --l2dtlb_assoc=$assoc
        mv m5out/stats.txt ./CS510-assignments/final-project/part1/bzip2/$assoc-way-$size-stats.txt
    done
done

