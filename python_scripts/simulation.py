import copy
import numpy as np

from commonroad.scenario.trajectory import Trajectory, InitialState
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.geometry.shape import Rectangle

from .planner import Planner
from .sensor import Sensor
from .occlusion_tracker import OcclusionTracker

from tqdm import tqdm

def step_scenario(scenario):
    new_scenario = copy.deepcopy(scenario)
    for vehicle in scenario.dynamic_obstacles:
        new_scenario.remove_obstacle(vehicle)
        if len(vehicle.prediction.trajectory.state_list) > 1:
            stepped_vehicle = step_vehicle(vehicle)
            new_scenario.add_objects(stepped_vehicle)
    return new_scenario


def step_vehicle(vehicle):
    initial_state = vehicle.prediction.trajectory.state_list[0]
    trajectory = Trajectory(1 + initial_state.time_step,
                            vehicle.prediction.trajectory.state_list[1:])
    return DynamicObstacle(vehicle.obstacle_id,
                           vehicle.obstacle_type,
                           vehicle.obstacle_shape,
                           initial_state,
                           TrajectoryPrediction(trajectory, vehicle.obstacle_shape))


def step_simulation(scenario, configuration, occlusion_aware= True):
    driven_state_list = []
    percieved_scenarios = []
    sensor_views = []
    emergency_brakes = []

    ideal_tracking = configuration.get('occlusion_params').get('ideal_tracking_enabled')

    ego_shape = Rectangle(configuration.get('vehicle_length'),
                          configuration.get('vehicle_width'))
    ego_initial_state = InitialState(position=np.array([configuration.get('initial_state_x'),
                                                        configuration.get('initial_state_y')]),
                                     orientation=configuration.get(
        'initial_state_orientation'),
        velocity=configuration.get(
        'initial_state_velocity'),
        time_step=0)

    ego_vehicle = DynamicObstacle(scenario.generate_object_id(),
                                  ObstacleType.CAR, ego_shape,
                                  ego_initial_state)
    
    sensor_position = [ego_vehicle.initial_state.position[0] + ego_vehicle.obstacle_shape.length/2, ego_vehicle.initial_state.position[1]]
    sensor = Sensor(sensor_position,
                    field_of_view=configuration.get(
                        'field_of_view_degrees')*2*np.pi/360,
                    min_resolution=configuration.get('min_resolution'),
                    view_range=configuration.get('view_range'))

    # We need the initial sensor view for initialising Occlusion Tracker
    sensor_view = sensor.get_sensor_view(scenario)

    occlusion_params = configuration.get('occlusion_params')
    occ_track = OcclusionTracker(scenario, sensor_view, occlusion_params, configuration.get('planning_horizon'))

    planner = Planner(ego_vehicle.initial_state,
                      vehicle_shape=ego_vehicle.obstacle_shape,
                      goal_point=[configuration.get('goal_point_x'),
                                  configuration.get('goal_point_y')],
                      reference_speed=configuration.get('reference_speed'),
                      max_acceleration=configuration.get('max_acceleration'),
                      max_deceleration=configuration.get('max_deceleration'),
                      time_horizon=configuration.get('planning_horizon'),
                      dt = scenario.dt)

    simulation_steps = configuration.get('simulation_duration')
    for step in tqdm(range(simulation_steps+1), desc=str(scenario.scenario_id),position=1,leave=False):
        if ideal_tracking and occlusion_aware:
            percieved_scenario = copy.deepcopy(scenario)
            sensor.update(ego_vehicle)
            sensor_view = sensor.get_sensor_view(scenario)
        else:
            # Start with an empty percieved scenario
            percieved_scenario = copy.deepcopy(scenario)
            for obstacle in percieved_scenario.obstacles:
                percieved_scenario.remove_obstacle(obstacle)

            # Update the sensor and get the sensor view and the list of observed obstacles
            # initial_state is current state
            sensor.update(ego_vehicle)
            sensor_view = sensor.get_sensor_view(scenario)
            observed_obstacles, _ = sensor.get_observed_obstacles(
                sensor_view, scenario)
            percieved_scenario.add_objects(observed_obstacles)

        # Update the tracker with the new sensor view and get the prediction for the shadows
        occ_track.update(sensor_view, ego_vehicle.initial_state.time_step)
        shadow_obstacles = occ_track.get_dynamic_obstacles(percieved_scenario)
        percieved_scenario.add_objects(shadow_obstacles)

        # Update the planner and plan a trajectory
        # Optionally, generate a no-stop zone on intersections to avoid having trajectories that stop on an intersection.
        planner.update(ego_vehicle.initial_state)
        collision_free_trajectory, emergency_stop = planner.plan(percieved_scenario)
        if collision_free_trajectory:
            ego_vehicle.prediction = collision_free_trajectory
        # else, if no trajectory found, keep previous collision free trajectory

        if step == 0 and emergency_stop:
           pass
           #raise Exception("Scenario starts with emergency-brake!")

        # Add the ego vehicle to the perceived scenario
        percieved_scenario.add_objects(ego_vehicle)

        percieved_scenarios.append(percieved_scenario)
        sensor_views.append(sensor_view)
        driven_state_list.append(ego_vehicle.initial_state)
        emergency_brakes.append(emergency_stop)

        ego_vehicle = step_vehicle(ego_vehicle)
        scenario = step_scenario(scenario)

    # Set initial_state to initial state and not current
    ego_vehicle.initial_state = driven_state_list.pop(0)
    driven_trajectory = Trajectory(1, driven_state_list)
    driven_trajectory_pred = TrajectoryPrediction(
        driven_trajectory, ego_vehicle.obstacle_shape)
    ego_vehicle.prediction = driven_trajectory_pred
    return ego_vehicle, percieved_scenarios, sensor_views, occ_track.get_shadows(), emergency_brakes, occ_track.get_computational_time()
