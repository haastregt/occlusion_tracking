# OCCLUSION TRACKING
Tracking the position and velocity of hidden traffic by reasoning about previous observations, in order to allow for less conservative trajectory planning in autonomous driving while maintaining safety guarantees

This project uses the python commonroad library and the c++ library CGAL for the reachability computations. In order for the code to run, a custom python library called py_occlusions is used to interface the python code with the c++ reachability code.

## INSTALL AND DEPENDENCIES
You can clone this repo anywhere you want by running $ git clone https://github.com/haastregt/occlusion_tracking.git in the directory where you want it.
Build the docker image and run compile_on_docker.sh to compile the code. Then the jupyter notebook can be used to run the demos

TODO write clear installation steps
