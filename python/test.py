from matplotlib import animation
import matplotlib.pyplot as plt
import yaml
from visualizer import Visualizer
from simulation import step_simulation
from commonroad.common.file_reader import CommonRoadFileReader
import sys
sys.path.insert(0, 'python/')


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


with open("scenario_building/config_recreated.yaml") as file:
    config = yaml.load(file, Loader=yaml.FullLoader)
scenario1, _ = CommonRoadFileReader(
    "scenario_building/DEU_Ffb-1_4_recreation.xml").open()
scenario2, _ = CommonRoadFileReader(
    "scenario_building/DEU_Ffb-1_4_recreation.xml").open()

track_vehicle, tracked_scenarios, tracked_views = step_simulation(
    scenario1, config)


# from occlusion_tracker import OcclusionTracker
# from commonroad.scenario.scenario import Scenario
# from shapely.geometry import Polygon

# point1 = [-10, 10]
# point2 = [10, 10]
# point3 = [10, -10]
# point4 = [-10, -10]

# point5 = [-20, 5]
# point6 = [15, 5]
# point7 = [15, -5]
# point8 = [-20, -5]

# point9 = [0, 0]
# point10 = [0, 15]
# point11 = [15, 15]
# point12 = [15, -15]
# point13 = [-10, -15]

# poly1 = Polygon([point1, point2, point3, point4])
# poly2 = Polygon([point5, point6, point7, point8])
# poly3 = Polygon([point9, point10, point11, point12, point13])

# polylist = [poly2, poly3, poly1]
# scenario = Scenario(dt=0.1)
# tracker = OcclusionTracker(scenario, poly1)
# print("Wow, we didn't get any errors")
# sensor_view2 = Polygon([[-10, 0], [10, 0], [10, -10], [-10, -10]])
# tracker.update(sensor_view2, 1)
# print("Wow, we didn't get any errors")

# output = tracker.get_cr_dynamic_obstacles(scenario)
# print(output)

# print("Wow, we didn't get any errors")
