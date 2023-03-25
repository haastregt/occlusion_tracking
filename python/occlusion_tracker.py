import numpy as np

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import InitialState
from commonroad.prediction.prediction import SetBasedPrediction, Occupancy

from utilities import ShapelyPolygon2Polygon
from shapely.geometry import Polygon

from py_occlusions import ReachabilityParams, OcclusionHandler


class OcclusionTracker:
    time_step: int

    # TODO: Make this load from yaml file
    params = ReachabilityParams()
    params.vmin = 0
    params.vmax = 10
    params.amin = -5
    params.amax = 7
    params.prediction_dt = 0.1
    params.prediction_horizon = 10
    params.min_shadow_volume = 1.0

    def __init__(self, scenario, sensor_view, initial_time_step=0):
        self.time_step = initial_time_step

        # TODO: Find all driving corridors
        point1 = [-10, 10]
        point2 = [10, 10]
        point3 = [10, -10]
        point4 = [-10, -10]

        point5 = [-20, 5]
        point6 = [15, 5]
        point7 = [15, -5]
        point8 = [-20, -5]

        point9 = [0, 0]
        point10 = [0, 15]
        point11 = [15, 15]
        point12 = [15, -15]
        point13 = [-10, -15]

        poly1 = Polygon([point1, point2, point3, point4])
        poly2 = Polygon([point5, point6, point7, point8])
        poly3 = Polygon([point9, point10, point11, point12, point13])

        driving_corridor_polygons = [poly2, poly3, poly1]

        self.occlusion_handler = OcclusionHandler(
            driving_corridor_polygons, sensor_view, self.time_step, self.params)

    def update(self, sensor_view, new_time_step):
        self.occlusion_handler.update(sensor_view, new_time_step)

    def get_cr_dynamic_obstacles(self, scenario):
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
