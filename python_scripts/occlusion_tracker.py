import numpy as np

from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.scenario import Lanelet
from commonroad.scenario.trajectory import InitialState
from commonroad.prediction.prediction import SetBasedPrediction, Occupancy

from shapely import Polygon

from .utilities import ShapelyPolygon2Polygon, ShapelyRemoveDoublePoints, create_dc_shapes

from py_occlusions import ReachabilityParams, OcclusionHandler


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

    def __init__(self, scenario, sensor_view, config, planning_horizon, initial_time_step=0):
        self.time_step = initial_time_step
        self.planning_horizon = planning_horizon
        self.load_params(config)
        self.ideal_tracking_enabled = config.get('ideal_tracking_enabled')
        
        dc, mapped_dc, lanes = create_dc_shapes(scenario.lanelet_network)
        sensor_view_processed = ShapelyRemoveDoublePoints(sensor_view, 0.1)

        self.occlusion_handler = OcclusionHandler(
            dc, mapped_dc, lanes, sensor_view_processed, self.time_step, self.params)
        
        if self.ideal_tracking_enabled:
            # Then we need to save the dc and lanes
            self.dc = dc
            self.lanes = lanes

    def update(self, sensor_view, new_time_step):
        self.time_step = new_time_step
        
        if self.ideal_tracking_enabled:
            # Then all traffic is already available in the scenario
            return

        sensor_view_processed = ShapelyRemoveDoublePoints(sensor_view, 0.1)

        self.occlusion_handler.update(sensor_view_processed, new_time_step)

    def get_dynamic_obstacles(self, scenario):
        if self.ideal_tracking_enabled:
            occupancy_sets = []
            for obstacle in scenario.dynamic_obstacles:
                position = obstacle.initial_state.position
                orientation = obstacle.initial_state.orientation
                w = obstacle.obstacle_shape.width/2
                l = obstacle.obstacle_shape.length/2
                x1 = position[0] + l*np.cos(orientation) - w*np.sin(orientation)
                y1 = position[1] + l*np.sin(orientation) + w*np.cos(orientation)
                x2 = position[0] - l*np.cos(orientation) - w*np.sin(orientation)
                y2 = position[1] - l*np.sin(orientation) + w*np.cos(orientation)
                x3 = position[0] - l*np.cos(orientation) + w*np.sin(orientation)
                y3 = position[1] - l*np.sin(orientation) - w*np.cos(orientation)
                x4 = position[0] + l*np.cos(orientation) + w*np.sin(orientation)
                y4 = position[1] + l*np.sin(orientation) - w*np.cos(orientation)
                initial_occupancy = Polygon([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
                velocity = obstacle.initial_state.velocity
                occupancy_sets.append(OcclusionHandler.propagate_known_obstacle(
                    self.dc[0], self.lanes[0], initial_occupancy, velocity, self.params))
        else:
            occupancy_sets = self.occlusion_handler.get_reachable_sets()

        dynamic_obstacles = []
        for occupancy_set in occupancy_sets:
            occupancies = []
            # First element is the shape of the occlusion itself
            for i, polygon in enumerate(occupancy_set[1:]):
                for k in range(self.params.prediction_interval):
                    occupancy = Occupancy(self.time_step+i*self.params.prediction_interval+k+1,
                                          ShapelyPolygon2Polygon(polygon))
                    occupancies.append(occupancy)

 
            for i in range(self.params.prediction_horizon, self.planning_horizon):
                occupancy_set.append(occupancy)

            if not occupancies:
                continue

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
        if self.ideal_tracking_enabled:
            return None
        
        return self.occlusion_handler.export_shadows()
    
    def get_computational_time(self):
        if self.ideal_tracking_enabled:
            return [[],[]]
        
        return self.occlusion_handler.export_computational_time()