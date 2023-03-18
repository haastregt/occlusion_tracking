# occlusion_tracking
Tracking the position and velocity of hidden traffic by reasoning about previous observations, in order to allow for less conservative trajectory planning in autonomous driving while maintaining safety guarantees

This project uses the python commonroad library and the c++ library CGAL for the reachability computations. In order for the code to run, a custom python library called py_occlusions has to be built to interface the python code with the c++ reachability code.
In order to achieve this, run the setup.sh script.

The following dependencies are required (now mostly for myself, update later to show what these are and where to get them):
- gcc and g++
- cmake and make
- boost (sudo apt-get install libboost-all-dev)
- GMP and MPFR (sudo apt-get install libgmp-dev libmpfr-dev)
- Qt5 (sudo apt-get install qt5-default)
- CGAL (sudo apt-get libcgal-dev libcgal-qt5-dev)
- pybind11 (pip install pybind11[global])
- commonroad-io (pip install commonroad-io)
