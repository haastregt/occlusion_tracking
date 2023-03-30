import numpy as np

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Lanelet
from commonroad.scenario.trajectory import InitialState
from commonroad.prediction.prediction import SetBasedPrediction, Occupancy

from utilities import Lanelet2ShapelyPolygon, ShapelyPolygon2Polygon, ShapelyRemoveDoublePoints
from shapely.geometry import Polygon

from py_occlusions import ReachabilityParams, OcclusionHandler

import matplotlib.pyplot as plt


class OcclusionTracker:
    time_step: int

    # TODO: Make this load from yaml file
    params = ReachabilityParams()
    params.vmin = 0
    params.vmax = 10
    params.amin = -5
    params.amax = 7
    params.prediction_dt = 0.5
    params.prediction_horizon = 4
    params.min_shadow_volume = 1.0

    def __init__(self, scenario, sensor_view, initial_time_step=0):
        print("Initialising Occlusion Tracker")
        self.time_step = initial_time_step

        # Find the initial lanelets
        initial_lanelets = []
        for lanelet in scenario.lanelet_network.lanelets:
            if lanelet.predecessor == []:
                initial_lanelets.append(lanelet)

        # Generate lanes (Collection of lanelets from start to end of the scenario)
        lanes = []
        for lanelet in initial_lanelets:
            current_lanes, _ = Lanelet.all_lanelets_by_merging_successors_from_lanelet(
                lanelet, scenario.lanelet_network, max_length=500)
            for lane in current_lanes:
                lanelet_shapely = Lanelet2ShapelyPolygon(lane)
                lanelet_processed = ShapelyRemoveDoublePoints(
                    lanelet_shapely, 0.1)
                lanes.append(lanelet_processed)

        # print("The lanes are: ")
        # for lane in lanes:
        #     if not lane.is_simple:
        #         print("Lanelet has self_intersection!")
        #     print(lane.exterior.xy)
        #     x, y = lane.exterior.xy
        #     plt.plot(x, y)
        #     for i in range(len(x)):
        #         plt.text(x[i], y[i], str(i))
        #     plt.show()

        # print("Our sensor view is: ")
        # if not sensor_view.is_simple:
        #     print("Sensor-view has self_intersection!")
        # plt.plot(*sensor_view.exterior.xy)
        # plt.show()

        sensor_view_processed = ShapelyRemoveDoublePoints(sensor_view, 0.1)

        self.occlusion_handler = OcclusionHandler(
            lanes, sensor_view_processed, self.time_step, self.params)

    def update(self, sensor_view, new_time_step):
        sensor_view_processed = ShapelyRemoveDoublePoints(sensor_view, 0.1)

        self.occlusion_handler.update(sensor_view, new_time_step)

    def get_dynamic_obstacles(self, scenario):
        occupancy_sets = self.occlusion_handler.get_reachable_sets()

        dynamic_obstacles = []
        for occupancy_set in occupancy_sets:
            occupancies = []
            # First element is the shape of the occlusion itself
            for i, polygon in enumerate(occupancy_set[1:]):
                occupancy = Occupancy(self.time_step+i+1,
                                      ShapelyPolygon2Polygon(polygon))
                occupancies.append(occupancy)

            obstacle_id = scenario.generate_object_id()
            obstacle_type = ObstacleType.UNKNOWN
            obstacle_shape = ShapelyPolygon2Polygon(occupancy_set[0])
            obstacle_initial_state = InitialState(position=np.array([0, 0]),
                                                  velocity=0,
                                                  orientation=0,
                                                  time_step=self.time_step)
            obstacle_prediction = SetBasedPrediction(
                self.time_step+1, occupancies)
            dynamic_obstacle = DynamicObstacle(obstacle_id,
                                               obstacle_type,
                                               obstacle_shape,
                                               obstacle_initial_state,
                                               obstacle_prediction)

            dynamic_obstacles.append(dynamic_obstacle)

        return dynamic_obstacles
