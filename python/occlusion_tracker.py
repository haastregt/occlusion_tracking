import numpy as np

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import InitialState
from commonroad.prediction.prediction import SetBasedPrediction, Occupancy

from utilities import ShapelyPolygon2Polygon

from py_occlusions import ReachabilityParams, OcclusionHandler

class OcclusionTracker:
    time_step: int

    #TODO: Make this load from yaml file
    params = ReachabilityParams()
    params.vmin = 0
    params.vmax = 10
    params.amin = -5
    params.amax = 7
    params.dt = 0.1
    params.prediction_horizon = 10
    params.min_shadow_volume = 1.0

    def __init__(self, scenario, sensor_view, initial_time_step=0):
        self.time_step = initial_time_step
        
        # TODO: Find all driving corridors
        driving_corridor_polygons = []
        
        self.occlusion_handler = OcclusionHandler(driving_corridor_polygons,sensor_view, self.time_step, self.params)

    def update(self, sensor_view, new_time_step):
        # TODO: Check if this value change is actually propagated to the cpp code 
        # (it should, because it is passed by reference but maybe pybind does something)
        self.params.dt = new_time_step - self.time_step
        self.time_step = new_time_step

        self.occlusion_handler.update(sensor_view)

    def get_cr_dynamic_obstacles(self, scenario):
        occupancy_sets = self.occlusion_handler.get_reachable_sets()

        dynamic_obstacles = []
        for occupancy_set in occupancy_sets:

            occupancies = []
            # First element is the shape of the occlusion itself
            for i, polygon in enumerate(occupancy_set[1:]):
                occupancy = Occupancy(self.time_step+i+1, ShapelyPolygon2Polygon(polygon))
                occupancies.append(occupancy)

            obstacle_id = scenario.generate_object_id()
            obstacle_type = ObstacleType.UNKNOWN
            obstacle_shape = ShapelyPolygon2Polygon(occupancy_set[0])
            obstacle_initial_state = InitialState(position = np.array([0,0]),
                                                  velocity = 0,
                                                  orientation = 0,
                                                  time_step = self.time_step)
            obstacle_prediction = SetBasedPrediction(self.time_step+1, occupancies)
            dynamic_obstacle = DynamicObstacle(obstacle_id,
                                               obstacle_type,
                                               obstacle_shape,
                                               obstacle_initial_state,
                                               obstacle_prediction)
            
            dynamic_obstacles.append(dynamic_obstacle)
        
        return dynamic_obstacles