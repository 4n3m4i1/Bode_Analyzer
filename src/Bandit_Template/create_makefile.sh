#!/bin/bash
mkdir build
cd build
cmake -DFAMILY=rp2040 ..
rm ./CMakeFiles/Bandit_Template.dir/flags.make
cp ../tempFLAGS.make ./CMakeFiles/Bandit_Template.dir/flags.make
