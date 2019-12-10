#!/bin/bash

# Script to clean the tree from all compiled files.
# You can rebuild them afterwards using "build.sh"
#
# Written by James Medel, 12/9/2019

# Go into directory where this bash script is contained
cd `dirname $0`

# Remove the dedicated output directories
rm -rf build

# We're done!
echo Cleaned up the project!