import numpy as np

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Lanelet
from commonroad.scenario.trajectory import InitialState
from commonroad.prediction.prediction import SetBasedPrediction, Occupancy

from .utilities import Lanelet2ShapelyPolygon, ShapelyPolygon2Polygon, ShapelyRemoveDoublePoints, create_lane_shapes, create_dc_shapes

from py_occlusions import ReachabilityParams, OcclusionHandler

import matplotlib.pyplot as plt

from shapely.geometry import Polygon


class OcclusionTracker:
    time_step: int

    def load_params(self, config):
        params = ReachabilityParams()
        params.vmin = config.get('vmin')
        params.vmax = config.get('vmax')
        params.amin = config.get('amin')
        params.amax = config.get('amax')
        params.phi = config.get('phi')
        params.dt = config.get('dt')
        params.prediction_interval = config.get('prediction_interval')
        params.prediction_horizon = config.get('prediction_horizon')
        params.min_shadow_volume = config.get('min_shadow_volume')
        params.mapping_quality = config.get('mapping_quality')
        params.simplification_precision = config.get('simplification_precision')
        params.requires_mapping = config.get('requires_mapping')
        params.velocity_tracking_enabled = config.get(
            'velocity_tracking_enabled')
        params.export_shadows = config.get('save_shadows')
        self.params = params

    def __init__(self, scenario, sensor_view, params, planning_horizon, initial_time_step=0):
        self.time_step = initial_time_step
        self.planning_horizon = planning_horizon
        self.load_params(params)

        # Find the initial lanelets
        # initial_lanelets = []
        # for lanelet in scenario.lanelet_network.lanelets:
        #     if lanelet.predecessor == []:
        #         initial_lanelets.append(lanelet)

        # Generate lanes (Collection of lanelets from start to end of the scenario)
        # lanes = []
        # mapped_lanes = []
        # for lanelet in initial_lanelets:
        #     current_lanes, _ = Lanelet.all_lanelets_by_merging_successors_from_lanelet(
        #         lanelet, scenario.lanelet_network, max_length=500)
        #     for lane in current_lanes:
        #         original, mapped = create_lane_shapes(lane)
        #         lanes.append(original)
        #         mapped_lanes.append(mapped)
        
        # lanes, mapped_lanes = create_dc_shapes(scenario.lanelet_network)
        dc, mapped_dc, lanes = create_dc_shapes(scenario.lanelet_network)

        sensor_view_processed = ShapelyRemoveDoublePoints(sensor_view, 0.1)

        self.occlusion_handler = OcclusionHandler(
            dc, mapped_dc, lanes, sensor_view_processed, self.time_step, self.params)

    def update(self, sensor_view, new_time_step):
        self.time_step = new_time_step
        sensor_view_processed = ShapelyRemoveDoublePoints(sensor_view, 0.1)

        self.occlusion_handler.update(sensor_view_processed, new_time_step)

    def get_dynamic_obstacles(self, scenario):
        occupancy_sets = self.occlusion_handler.get_reachable_sets()

        dynamic_obstacles = []
        for occupancy_set in occupancy_sets:
            # print("Number of occupancies: ", len(occupancy_set))
            # plt.figure()
            # for polygon in occupancy_set:
            #     plt.plot(*polygon.exterior.xy)
            # plt.show()

            occupancies = []
            # First element is the shape of the occlusion itself
            for i, polygon in enumerate(occupancy_set[1:]):
                for k in range(self.params.prediction_interval):
                    occupancy = Occupancy(self.time_step+i*self.params.prediction_interval+k+1,
                                          ShapelyPolygon2Polygon(polygon))
                    occupancies.append(occupancy)

 
            for i in range(self.params.prediction_horizon, self.planning_horizon):
                occupancy_set.append(occupancy)

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

    def get_shadows(self):
        return self.occlusion_handler.export_shadows()
    
    def get_computational_time(self):
        return self.occlusion_handler.export_computational_time()