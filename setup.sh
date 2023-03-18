#!/bin/bash

python_exec_path=$( which python3 )
echo Python executable path: $python_exec_path

cd occlusion_package
mkdir -p build
cd build

cmake .. -DPYTHON_LIBRARY_DIR=$PYTHONPATH -DPYTHON_EXECUTABLE=$python_exec_path
make
sudo make install