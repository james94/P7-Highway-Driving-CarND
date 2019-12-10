#!/bin/bash

# Script to build all components from scratch, using the max available CPU Power

# Given parameters are passed over to CMake
# Examples:
#       * ./build_all.sh -DCMAKE_BUILD_TYPE=Debug
#       * ./build_all.sh VERBOSE=1
#
# Written by James Medel, 12/9/2019

# Go into directory where this bash script is contained
cd `dirname $0`

# Compile code
mkdir -p build
cd build
cmake ..
make $*