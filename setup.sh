#!/bin/bash
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

python_exec_path=$( which python3 )
echo Python executable path: $python_exec_path
echo Python library path: $PYTHONPATH

cd occlusion_package
mkdir -p build
cd build

cmake .. -DPYTHON_LIBRARY_DIR=$PYTHONPATH -DPYTHON_EXECUTABLE=$python_exec_path
make
sudo make install