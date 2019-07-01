#!/usr/bin/env bash

# check if the user is running the script with 'source' rather than executing directly.
# If executed directly, the 'source devel/setup.bash' won't work as expected
(return 0 2>/dev/null) && sourced=1 || sourced=0

if [ "$sourced" == "0" ]; then
    echo "Run this script with 'source ./rebuild.sh'"
else
    rm -r build/
    rm -r devel/
    catkin_make clean
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so -DCMAKE_BUILD_TYPE=Debug
    source devel/setup.bash
fi
