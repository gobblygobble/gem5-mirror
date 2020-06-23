#!/bin/bash

build/X86/gem5.opt configs/SPEC2006/speccpu2006-part1.py --benchmark=soplex --cpu-type=MinorCPU --caches --l2dtlb_size=128
mv m5out/stats.txt ./CS510-assignments/final-project/part1/soplex/2-way-128-stats.txt

build/X86/gem5.opt configs/SPEC2006/speccpu2006-part1.py --benchmark=soplex --cpu-type=MinorCPU --caches --l2dtlb_size=256
mv m5out/stats.txt ./CS510-assignments/final-project/part1/soplex/2-way-256-stats.txt

build/X86/gem5.opt configs/SPEC2006/speccpu2006-part1.py --benchmark=soplex --cpu-type=MinorCPU --caches --l2dtlb_size=512
mv m5out/stats.txt ./CS510-assignments/final-project/part1/soplex/2-way-512-stats.txt

build/X86/gem5.opt configs/SPEC2006/speccpu2006-part1.py --benchmark=soplex --cpu-type=MinorCPU --caches --l2dtlb_size=1024
mv m5out/stats.txt ./CS510-assignments/final-project/part1/soplex/2-way-1024-stats.txt
