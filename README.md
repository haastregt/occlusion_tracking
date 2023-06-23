# OCCLUSION TRACKING WITH VELOCITY BOUNDS
Tracking the position and velocity of hidden traffic by reasoning about previous observations, in order to allow for less conservative trajectory planning in autonomous driving while maintaining safety guarantees

This project uses the python commonroad library and the c++ library CGAL for the reachability computations. In order for the code to run, a custom python library called py_occlusions is used to interface the python simulation code with the reachability computations in c++.

## INSTALL AND DEPENDENCIES
First, install commonroad and pandas:
- `$ pip install commonroad-all`
- `$ pip install pandas`

Then clone this repo anywhere you want using `$ git clone https://github.com/haastregt/occlusion_tracking.git` in the directory where you want it.

If you want to use the code as-is, you can move the file `occlusion_tracking/occlusion_package/install/py_occlusions.cpython-38-x86_64-linux-gnu.so` to your python libraries folder, which is likely located at `~/.local/lib/python3.8/site-packages`. You can now run any of the notebooks that are included in this repo. 

## MODIFYING CODE
If you want to change the reachability code, you will need to rebuild the python interface. You can do so as follows:
- Build the Docker image by running `$ docker build .` from the root directory of this repo.
- In the file `compile_package.sh`, make sure to set the correct path for your python libraries folder, and save it.
- When you are ready to compile the code, run `$ sudo compile_on_docker.sh` from the root directory of this repo. You can now run any of the notebooks with updated code.

## REPRODUCING RESULTS
The scenarios used are retrieved from the HighD dataset. This dataset is not public but can be requested at https://www.highd-dataset.com/. The retrieved scenarios could not be shared here, so only the results are saved in the pickle file. If you want to reproduce the results, you will need to request the dataset, and can then extract the simulated scenarios as follows:
- Navigate to `occlusion_tracking/scenarios/scenario_setup`
- Run `$ python3 create_scenarios.py [path_to_dataset] ../`
  You can now run the script `simulate_all_scenarios.py` located in the root folder of this repo to resimulate all scenarios.

The four scenarios shown in the thesis do have information considering the scenario, and can be animated. These results files are stored seperately in the results folder.
