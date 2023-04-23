import sys

sys.path.insert(0, 'python/')

from commonroad.common.file_reader import CommonRoadFileReader
from simulation import step_simulation
from visualizer import Visualizer
import yaml
import matplotlib.pyplot as plt
from matplotlib import animation
from utilities import find_RSS_distance


def plot(time_step, ego_vehicle, scenarios, sensor_views):
    plt.cla()
    Visualizer().plot(scenario=scenarios[time_step],
                      sensor_view=sensor_views[time_step],
                      ego_vehicle=scenarios[time_step].obstacle_by_id(
                          ego_vehicle.obstacle_id),
                      time_begin=time_step)
    plt.axis('scaled')
    plt.xlim(0, 120)
    plt.ylim(-40, 40)


with open("scenarios/config_recreated.yaml") as file:
    config = yaml.load(file, Loader=yaml.FullLoader)
scenario1, _ = CommonRoadFileReader("scenarios/USA_US101-12_4_T-1.xml").open()
scenario2, _ = CommonRoadFileReader("scenarios/USA_US101-12_4_T-1.xml").open()

config['occlusion_params']['velocity_tracking_enabled'] = True
track_vehicle, tracked_scenarios, tracked_views, shadows  = step_simulation(scenario1, config)

for shadow in shadows:
    Visualizer().plot_3D_shadows(shadow, 15, 3)


config['occlusion_params']['velocity_tracking_enabled'] = False
no_track_vehicle, not_tracked_scenarios, not_tracked_views, shadows = step_simulation(scenario2, config)

dists = find_RSS_distance(no_track_vehicle, scenario1, config)
print(dists)

for shadow in shadows:
    Visualizer().plot_3D_shadows(shadow, 15, 3)
