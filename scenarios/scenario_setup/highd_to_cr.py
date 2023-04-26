import os
import copy
import math
import numpy as np
import pandas as pd

from typing import List
from enum import Enum
from pandas import DataFrame

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.scenario.scenario import Scenario, ScenarioID
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import InitialState, Trajectory
from commonroad.scenario.lanelet import Lanelet, LaneletType, RoadUser, LineMarking
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction

class Direction(Enum):
    """
    Enum for representing upper or lower interstate road
    """
    UPPER = 1
    LOWER = 2

obstacle_class_dict = {
    'Truck': ObstacleType.TRUCK,
    'Car': ObstacleType.CAR
}

AUTHOR = "Jonne"
AFFILIATION = "KTH Royal Institute of Technology"
SOURCE = "HighD Highway Drone Dataset"
TAGS = {}

def generate_single_scenario(ego_id: int, to_remove_ids: List[int], output_dir: str,
                             tracks_df: pd.DataFrame, tracks_meta_df: pd.DataFrame, meta_scenario: Scenario,
                             benchmark_id: str, direction: Direction, frame_start: int, frame_end: int,
                             obstacle_start_at_zero: bool, downsample: int):
    """
    Generate a single CommonRoad scenario based on hihg-D record snippet

    :param output_dir: path to store generated CommonRoad scenario files
    :param keep_ego: boolean indicating if vehicles selected for planning problem should be kept in scenario
    :param tracks_df: single track
    :param tracks_meta_df: single meta information track
    :param meta_scenario: CommonRoad scenario with lanelet network
    :param benchmark_id: CommonRoad benchmark ID for scenario
    :param direction: indicator for upper or lower road of interstate
    :param frame_start: start of frame in time steps of record
    :param frame_end: end of frame in time steps of record
    :param obstacle_start_at_zero: boolean indicating if the initial state of an obstacle has to start
    at time step zero
    :param downsample: resample states of trajectories of dynamic obstacles every downsample steps
    :return: None
    """

    def enough_time_steps(veh_id):
        # Need at least two frames, or to be existing upon start if obstacle_start_at_zero is true
        vehicle_meta = tracks_meta_df[tracks_meta_df.id == veh_id]
        if not obstacle_start_at_zero and frame_end - vehicle_meta.initialFrame.values[0] < 2 * downsample \
                or vehicle_meta.finalFrame.values[0] - frame_start < 2 * downsample:
            return False
        elif obstacle_start_at_zero and vehicle_meta.initialFrame.values[0] > frame_start \
            or vehicle_meta.finalFrame.values[0] - frame_start < 2 * downsample:
            return False
        return True

    # copy meta_scenario with lanelet networks
    scenario = copy.deepcopy(meta_scenario)
    scenario.scenario_id = ScenarioID.from_benchmark_id(benchmark_id, '2020a')

    # read tracks appear between [frame_start, frame_end]
    scenario_tracks_df = tracks_df[(tracks_df.frame >= frame_start) & (tracks_df.frame <= frame_end)]

    # generate CR obstacles
    for vehicle_id in [vehicle_id for vehicle_id in scenario_tracks_df.id.unique()
                       if vehicle_id in tracks_meta_df[tracks_meta_df.drivingDirection == direction.value].id.unique()]:
        # if appearing time steps < min_time_steps, skip vehicle
        if not enough_time_steps(vehicle_id):
            continue
        if vehicle_id == ego_id:
            continue
        if vehicle_id in to_remove_ids:
            continue
        
        #print("Generating scenario {}, vehicle id {}".format(benchmark_id, vehicle_id), end="\r")
        do = generate_dynamic_obstacle(scenario, vehicle_id, tracks_meta_df, scenario_tracks_df, frame_start, downsample)
        scenario.add_objects(do)

    # rotate scenario if it is upper scenario
    if direction == Direction.UPPER:
        translation = np.array([0.0, 0.0])
        angle = np.pi
        scenario.translate_rotate(translation, angle)

    # write new scenario
    fw = CommonRoadFileWriter(scenario, PlanningProblemSet(), author=AUTHOR, affiliation=AFFILIATION, source=SOURCE, tags=TAGS)

    filename = os.path.join(output_dir, "{}.xml".format(scenario.scenario_id))
    fw.write_to_file(filename, OverwriteExistingFile.ALWAYS, check_validity=obstacle_start_at_zero)
    print("Scenario file stored in {}".format(filename))


def resample_polyline(polyline, step=2.0):
    new_polyline = [polyline[0]]
    current_position = 0 + step
    current_length = np.linalg.norm(polyline[0] - polyline[1])
    current_idx = 0
    while current_idx < len(polyline) - 1:
        if current_position >= current_length:
            current_position = current_position - current_length
            current_idx += 1
            if current_idx > len(polyline) - 2:
                break
            current_length = np.linalg.norm(
                polyline[current_idx + 1] - polyline[current_idx]
            )
        else:
            rel = current_position / current_length
            new_polyline.append(
                (1 - rel) * polyline[current_idx] + rel * polyline[current_idx + 1]
            )
            current_position += step
    return np.array(new_polyline)


def get_meta_scenario(dt: float, benchmark_id: str, lane_markings: List[float], speed_limit: float,
                      road_length: int, direction: Direction, road_offset: int, num_vertices: int = 10):
    """
    Generates meta CommonRoad scenario containing only lanelet network

    :param dt: time step size
    :param benchmark_id: benchmark ID of meta scenario
    :param lane_markings: list of y-positions for lane markings
    :param speed_limit: speed limits for road
    :param road_length: length of road
    :param direction: indicator for upper or lower interstate road
    :param road_offset: length added on both sides of road
    :return: CommonRoad scenario
    """
    scenario = Scenario(dt, ScenarioID.from_benchmark_id(benchmark_id, "2020a"))
    resample_step = (road_offset + 2 * road_offset) / num_vertices
    if direction is Direction.UPPER:
        for i in range(len(lane_markings) - 1):
            # get two lines of current lane
            lane_y = lane_markings[i]
            next_lane_y = lane_markings[i + 1]

            right_vertices = resample_polyline(
                np.array([[road_length + road_offset, lane_y], [-road_offset, lane_y]]), step= resample_step
            )
            left_vertices = resample_polyline(
                np.array([[road_length + road_offset, next_lane_y], [-road_offset, next_lane_y]]), step=resample_step
            )
            center_vertices = (left_vertices + right_vertices) / 2.0

             # assign lanelet ID and adjacent IDs and lanelet types
            lanelet_id = i + 1
            lanelet_type = {LaneletType.INTERSTATE, LaneletType.MAIN_CARRIAGE_WAY}
            adjacent_left = lanelet_id + 1
            adjacent_right = lanelet_id - 1
            adjacent_left_same_direction = True
            adjacent_right_same_direction = True
            line_marking_left_vertices = LineMarking.DASHED
            line_marking_right_vertices = LineMarking.DASHED

            if i == len(lane_markings) - 2:
                adjacent_left = None
                adjacent_left_same_direction = False
                line_marking_left_vertices = LineMarking.SOLID
            elif i == 0:
                adjacent_right = None
                adjacent_right_same_direction = False
                line_marking_right_vertices = LineMarking.SOLID

            # add lanelet to scenario
            scenario.add_objects(
                Lanelet(lanelet_id=lanelet_id, left_vertices=left_vertices,  right_vertices=right_vertices,
                        center_vertices=center_vertices, adjacent_left=adjacent_left,
                        adjacent_left_same_direction=adjacent_left_same_direction, adjacent_right=adjacent_right,
                        adjacent_right_same_direction=adjacent_right_same_direction, user_one_way={RoadUser.VEHICLE},
                        line_marking_left_vertices=line_marking_left_vertices,
                        line_marking_right_vertices=line_marking_right_vertices, lanelet_type=lanelet_type))
    else:
        for i in range(len(lane_markings) - 1):

            # get two lines of current lane
            next_lane_y = lane_markings[i + 1]
            lane_y = lane_markings[i]

            # get vertices of three lines
            # setting length 450 and -50 can cover all vehicle in this range
            left_vertices = resample_polyline(
                np.array([[-road_offset, lane_y], [road_length + road_offset, lane_y]]), step=resample_step
            )
            right_vertices = resample_polyline(
                np.array([[-road_offset, next_lane_y], [road_length + road_offset, next_lane_y]]), step=resample_step
            )
            center_vertices = (left_vertices + right_vertices) / 2.0

            # assign lane ids and adjacent ids
            lanelet_id = i + 1
            lanelet_type = {LaneletType.INTERSTATE, LaneletType.MAIN_CARRIAGE_WAY}
            adjacent_left = lanelet_id - 1
            adjacent_right = lanelet_id + 1
            adjacent_left_same_direction = True
            adjacent_right_same_direction = True
            line_marking_left_vertices = LineMarking.DASHED
            line_marking_right_vertices = LineMarking.DASHED

            if i == 0:
                adjacent_left = None
                adjacent_left_same_direction = False
                line_marking_left_vertices = LineMarking.SOLID
            elif i == len(lane_markings) - 2:
                adjacent_right = None
                adjacent_right_same_direction = False
                line_marking_right_vertices = LineMarking.SOLID

            # add lanelet to scenario
            scenario.add_objects(
                Lanelet(lanelet_id=lanelet_id, left_vertices=left_vertices,  right_vertices=right_vertices,
                        center_vertices=center_vertices, adjacent_left=adjacent_left,
                        adjacent_left_same_direction=adjacent_left_same_direction, adjacent_right=adjacent_right,
                        adjacent_right_same_direction=adjacent_right_same_direction, user_one_way={RoadUser.VEHICLE},
                        line_marking_left_vertices=line_marking_left_vertices,
                        line_marking_right_vertices=line_marking_right_vertices, lanelet_type=lanelet_type))

    return scenario


def get_velocity(track_df: DataFrame) -> np.array:
    """
    Calculates velocity given x-velocity and y-velocity

    :param track_df: track data frame of a vehicle
    :return: array of velocities for vehicle
    """
    return np.sqrt(track_df.xVelocity ** 2 + track_df.yVelocity ** 2)


def get_orientation(track_df: DataFrame) -> np.array:
    """
    Calculates orientation given x-velocity and y-velocity

    :param track_df: track data frame of a vehicle
    :return: array of orientations for vehicle
    """
    return np.arctan2(-track_df.yVelocity, track_df.xVelocity)


def get_acceleration(track_df: DataFrame) -> np.array:
    """
    Calculates acceleration given x-acceleration and y-acceleration

    :param track_df: track data frame of a vehicle
    :return: array of accelerations for vehicle
    """
    return np.sqrt(track_df.xAcceleration ** 2 + track_df.yAcceleration ** 2)


def generate_dynamic_obstacle(scenario: Scenario, vehicle_id: int, tracks_meta_df: DataFrame,
                              tracks_df: DataFrame, time_step_correction: int, downsample: int) -> DynamicObstacle:
    """

    :param scenario: CommonRoad scenario
    :param vehicle_id: ID of obstacle to generate
    :param tracks_meta_df: track meta information data frames
    :param tracks_df: track data frames
    :return: CommonRoad dynamic obstacle
    """

    vehicle_meta = tracks_meta_df[tracks_meta_df.id == vehicle_id]
    vehicle_tracks = tracks_df[tracks_df.id == vehicle_id]

    length = vehicle_meta.width.values[0]
    width = vehicle_meta.height.values[0]

    initial_time_step_cr = math.ceil((int(vehicle_tracks.frame.values[0]) - time_step_correction) / downsample)
    initial_time_step_cr = int(initial_time_step_cr)
    initial_frame = initial_time_step_cr * downsample
    dynamic_obstacle_id = scenario.generate_object_id()
    dynamic_obstacle_type = obstacle_class_dict[vehicle_meta['class'].values[0]]
    dynamic_obstacle_shape = Rectangle(width=width, length=length)

    xs = np.array(vehicle_tracks.x)
    ys = np.array(-vehicle_tracks.y)
    velocities = get_velocity(vehicle_tracks)
    orientations = get_orientation(vehicle_tracks)
    accelerations = get_acceleration(vehicle_tracks)

    state_list = []
    for cr_timestep, frame_idx in enumerate(range(0, xs.shape[0], downsample)):
        x = xs[frame_idx]
        y = ys[frame_idx]
        v = velocities.values[frame_idx]
        theta = orientations.values[frame_idx]
        a = accelerations.values[frame_idx]
        state_list.append(InitialState(position=np.array([x, y]), velocity=v, orientation=theta,
                                time_step=cr_timestep + initial_time_step_cr))

    dynamic_obstacle_initial_state = state_list[0]

    dynamic_obstacle_trajectory = Trajectory(initial_time_step_cr + 1, state_list[1:])
    dynamic_obstacle_prediction = TrajectoryPrediction(dynamic_obstacle_trajectory, dynamic_obstacle_shape)

    return DynamicObstacle(dynamic_obstacle_id, dynamic_obstacle_type, dynamic_obstacle_shape,
                           dynamic_obstacle_initial_state, dynamic_obstacle_prediction)
