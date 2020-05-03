#!/usr/bin/env bash

# check if the user is running the script with 'source' rather than executing directly.
# If executed directly, the 'source devel/setup.bash' won't work as expected
(return 0 2>/dev/null) && sourced=1 || sourced=0

if [ "$sourced" == "0" ]; then
    echo "Run this script with 'source ./rebuild.sh'"
else
    rm -r build/
    rm -r devel/
    
    CURRENT_DIR=$(pwd)
    CMAKE_MODULE_DIR=$CURRENT_DIR/cmake/Modules

    if [[ -f "/usr/lib/x86_64-linux-gnu/libpython3.6m.so" ]]; then
        PYTHON_LIBRARY="/usr/lib/x86_64-linux-gnu/libpython3.6m.so"
    else
        PYTHON_LIBRARY="/usr/lib/aarch64-linux-gnu/libpython3.6m.so"
    fi

    #catkin_make clean -DCMAKE_MODULE_PATH=$CMAKE_MODULE_DIR
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=$PYTHON_LIBRARY -DCMAKE_BUILD_TYPE=Debug -DCMAKE_MODULE_PATH=$CMAKE_MODULE_DIR

    # Force regen of the launch files only if on jetson
    isjetson=$(lscpu | grep "aarch64")
    if [ -n "$isjetson" ]
    then
        echo "regenerating launch files from mapped hardware."
        rm -r launch
        python3 utility-scripts/detect_hardware.py --generate-launch-files --strict
    fi

    # Regenerate the YCM compile flags database for cpp files
    python3 utility-scripts/generate_ycm_database.py

    source devel/setup.bash
fi
