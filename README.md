# occlusion_tracking
Tracking the position and velocity of hidden traffic by reasoning about previous observations, in order to allow for less conservative trajectory planning in autonomous driving while maintaining safety guarantees

This project uses the python commonroad library and the c++ library CGAL for the reachability computations. In order for the code to run, a custom python library called py_occlusions has to be built to interface the python code with the c++ reachability code.
In order to achieve this, run the setup.sh script.

The pre-compiled py_occlusions library can be found in /occlusions_package/install, but it has to be moved to your pythin site-packages. To see where this folder is located, you can run $ $PYTHONPATH

The following dependencies are required if you want to build the py_occlusions package yourself:
- gcc and g++
- cmake and make
- boost (sudo apt-get install libboost-all-dev)
- GMP and MPFR (sudo apt-get install libgmp-dev libmpfr-dev)
- Qt5 (sudo apt-get install qt5-default)
- CGAL (sudo apt-get libcgal-dev libcgal-qt5-dev)
- pybind11 (pip install pybind11[global])
- commonroad-io (pip install commonroad-io)

Then run $ ./setup.sh
