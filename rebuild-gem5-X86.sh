#!/bin/bash

rm -r build
scons build/X86/gem5.opt -j12
