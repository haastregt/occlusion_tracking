#!/bin/bash
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

cd occlusion_package
mkdir -p build
mkdir -p install

python_exec_path=$( which python3 )
echo Python executable path: $python_exec_path
PYTHONPATH="/home/jvarvn/.local/lib/python3.8/site-packages"
echo Python library path: $PYTHONPATH

sudo docker run -v /home/jvarvn/Documents/Implementations/occlusion_tracking/occlusion_package/build:/build             \
                -v /home/jvarvn/Documents/Implementations/occlusion_tracking/occlusion_package/install:/install         \
                -v /home/jvarvn/Documents/Implementations/occlusion_tracking/occlusion_package:/occlusion_package       \
                -t build-docker                                                                                         \
                bash -c "cmake ../occlusion_package -DPYTHON_LIBRARY_DIR=/install -DPYTHON_EXECUTABLE=$python_exec_path && make && make install"

cp -a ./install/. $PYTHONPATH/