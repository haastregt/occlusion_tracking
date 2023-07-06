#!/bin/bash

cd occlusion_package
mkdir -p build
mkdir -p install

occlusion_package_path=$( pwd )
echo Occlusion package path: $occlusion_package_path
python_exec_path=$( which python3 )
echo Python executable path: $python_exec_path
python_site_packages_path=$( python3 -c "import sys; print(next(p for p in sys.path if 'site-packages' in p))" )
echo Python library path: $python_site_packages_path

sudo docker run -v $occlusion_package_path/build:/build             \
                -v $occlusion_package_path/install:/install         \
                -v $occlusion_package_path:/occlusion_package       \
                -t cgal_552                                                                                             \
                bash -c "cmake ../occlusion_package -DPYTHON_LIBRARY_DIR=/install -DPYTHON_EXECUTABLE=$python_exec_path && make && make install"

cp -a ./install/. $python_site_packages_path/
