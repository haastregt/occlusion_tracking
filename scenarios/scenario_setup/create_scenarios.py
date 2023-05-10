"""
IN ORDER TO USE, ENTER THIS FOLDER IN COMMAND LINE AND RUN:
    python3 create_scenarios.py ../../datasets/data ../
"""

import os
import sys
import pandas as pd
from pandas import DataFrame
from typing import Union
import numpy as np
import glob
import yaml
from highd_to_cr import Direction, get_meta_scenario, generate_single_scenario

data_path = sys.argv[1]
scenario_path = sys.argv[2]

pd.options.mode.chained_assignment = None

REMOVE_ALL_OTHER_TRAFFIC = True

ROAD_LENGTH = 420  # For all scenarios
ROAD_OFFSET = 20  # To add before and after, avoiding traffic outside of lane

FPS = 25  # frames per second of the video
DESIRED_FREQ = 5  # desired simulation frequency
# highD framerate is 25 fps, we want to simulate at 5Hz
DOWNSAMPLING = int(FPS/DESIRED_FREQ)

TIME_BEFORE = 3.6  # Time before a lane change that the scenario begins
TIME_AFTER = 5.4  # Time after a lane change that the scenario ends

# m/s = 135 km/h. This is 120 km/h (most highways speed limits in europe) + 12,5% speeding margin
SPEED_LIMIT = 37.5
MAX_DEC = 4  # Maximum deceleration of ego to filter out any infeasible initial states
MAX_ACC_EGO = 2  # To detect legal merges
MAX_ACC_OTHER = 3  # To detect legal merges

total_scenarios = 0


def find_valid_scenarios(tracks_meta_df, tracks_df, video_meta_df):

    def has_valid_initial_state(initial_state):
        distance = np.abs(initial_state.dhw).values[0]
        if not distance:
            return True

        velocity = np.abs(initial_state.xVelocity).values[0]
        if 0.5*velocity**2/MAX_DEC < distance:
            return True

        return False

    def find_lane_change_interval(vehicle_df):
        vehicle_df['change'] = vehicle_df['laneId'].diff()
        change_frame = vehicle_df[vehicle_df['change'] != 0].frame.values[1]
        first_frame = change_frame - TIME_BEFORE*FPS
        last_frame = change_frame + TIME_AFTER*FPS
        return first_frame, last_frame, change_frame

    first_frames = []
    final_frames = []
    ego_ids = []
    remove_ids = []
    merging_ids = []

    if video_meta_df["speedLimit"].values[0] == -1:
        speed_limit = SPEED_LIMIT
    else:
        speed_limit = min([SPEED_LIMIT, 
                           1.125*video_meta_df["speedLimit"].values[0]])

    for vehicle_id in tracks_meta_df[tracks_meta_df.numLaneChanges == 1].id.values:
        # Check that vehicle is not above our artificial speed limit
        if tracks_meta_df[tracks_meta_df.id == vehicle_id].maxXVelocity.values[0] > speed_limit:
            continue

        # Check that there was a car behind (which will be the ego vehicle)
        first_frame, final_frame, change_frame = find_lane_change_interval(
            tracks_df[tracks_df.id == vehicle_id])
        ego_id = tracks_df[(tracks_df.id == vehicle_id) & (
            tracks_df.frame == change_frame)].followingId.values
        if not ego_id:
            continue
        ego_id = ego_id[0]

        # Check that the merging happens close enough to the vehicle, otherwise it is not really a cut-in
        if tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == change_frame)].dhw.values[0] > 100:
            continue

        # Check that ego and merging vehicle exist over the whole time interval
        if first_frame < 0:
            continue
        if len(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)].index) == 0:
            continue
        if len(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == final_frame)].index) == 0:
            continue
        if len(tracks_df[(tracks_df.id == vehicle_id) & (tracks_df.frame == first_frame)].index) == 0:
            continue
        if len(tracks_df[(tracks_df.id == vehicle_id) & (tracks_df.frame == final_frame)].index) == 0:
            continue

        # The ego vehicle should not have lane-changes itself to ensure it was actually a cut-in
        if tracks_meta_df[tracks_meta_df.id == ego_id].numLaneChanges.values[0] != 0:
            continue

        # For ease, just consider to direction to the right (we have cut-ins in abundance anyways)
        if tracks_meta_df[tracks_meta_df.id == ego_id].drivingDirection.values[0] == 1:
            continue

        # Initial state still assumes stationary vehicle, so on initial state need enough distance to be feasible
        if not has_valid_initial_state(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]):
            continue

        if REMOVE_ALL_OTHER_TRAFFIC:
            remove_id = list(
                tracks_df[tracks_df.frame == first_frame]["id"].unique())
            # Just keep ego and merging
            remove_id.remove(ego_id)
            remove_id.remove(vehicle_id)
        else:
            # Remove anything behind the ego vehicle
            remove_id = []
            all_ids = tracks_df[tracks_df.frame == first_frame]["id"].unique()
            for traffic_id in all_ids:
                if tracks_df[(tracks_df.frame == first_frame) & (tracks_df.id == traffic_id)]["x"].values[0] < tracks_df[(tracks_df.frame == first_frame) & (tracks_df.id == ego_id)]["x"].values[0]:
                    remove_id.append(traffic_id)

            # Sometimes the merging vehicle starts behind, but we want to keep this one
            try:
                remove_id.remove(vehicle_id)
            except:
                pass

        first_frames.append(first_frame)
        final_frames.append(final_frame)
        ego_ids.append(ego_id)
        remove_ids.append(remove_id)
        merging_ids.append(vehicle_id)

    return first_frames, final_frames, ego_ids, remove_ids, merging_ids


def generate_yaml(video_meta_df, tracks_df, tracks_meta_df, ego_id, merging_id, first_frame, final_frame, output_filename):
    if video_meta_df["speedLimit"].values[0] == -1:
        speed_limit = SPEED_LIMIT
    else:
        speed_limit = min(
            [SPEED_LIMIT, 1.125*video_meta_df["speedLimit"].values[0]])

    change_frame = first_frame + FPS*TIME_AFTER

    v_other = tracks_df[(tracks_df.id == merging_id) & (
        tracks_df.frame == change_frame)]["xVelocity"].values[0]
    v_ego = tracks_df[(tracks_df.id == ego_id) & (
        tracks_df.frame == change_frame)]["xVelocity"].values[0]
    merge_distance = tracks_df[(tracks_df.id == ego_id) & (
        tracks_df.frame == change_frame)]["dhw"].values[0]
    # safe merge distance: v_ego**2/2a_max_ego - v_other**2/2a_max_other.
    if v_ego**2/MAX_ACC_EGO - v_other**2/MAX_ACC_OTHER < merge_distance:
        safe_merge = True
    else:
        safe_merge = True

    if tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["x"].values[0] < tracks_df[(tracks_df.id == merging_id) & (tracks_df.frame == first_frame)]["x"].values[0]:
        overtake = False
    else:
        overtake = True

    v_ego_complete = tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame >= first_frame) & (
        tracks_df.frame <= final_frame)].sort_values(by=['frame'])["xVelocity"].values

    data = dict(
        simulation_duration         = int((final_frame - first_frame) // DOWNSAMPLING),

        initial_state_x             = float(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["x"].values[0]),
        initial_state_y             = float(-tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["y"].values[0]),
        initial_state_orientation   = 0,
        initial_state_velocity      = float(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["xVelocity"].values[0]),

        reference_velocity          = float(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["xVelocity"].values[0]),
        vmax                        = float(speed_limit),
        v_other                     = float(v_other),
        recorded_ego_velocity       = list(map(float, v_ego_complete)),

        legal_merge                 = bool(safe_merge),
        is_overtake                 = bool(overtake),
        merge_dhw                   = float(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == change_frame)]["dhw"].values[0]),
        merge_ttc                   = float(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == change_frame)]["ttc"].values[0]),

        planning_horizon            = int(max((tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["xVelocity"].values[0] + 4.99 + 1) // 5 * 5, 30)),
        # + 1 because passive safety braking traj is started at timestep k + 1

        vehicle_type                = tracks_meta_df[tracks_meta_df.id == ego_id]["class"].values[0],
        vehicle_length              = float(tracks_meta_df[tracks_meta_df.id == ego_id]["width"].values[0]),
        vehicle_width               = float(tracks_meta_df[tracks_meta_df.id == ego_id]["height"].values[0]),

        goal_point_x                = ROAD_LENGTH,
        goal_point_y                = float(-tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["y"].values[0])
    )
    with open(output_filename, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)


def get_lane_markings(recording_df: DataFrame, extend_width=2.):
    """
    Extracts upper and lower lane markings from data frame;
    extend width of the outter lanes because otherwise some vehicles are off-road at the first time step.

    :param recording_df: data frame of the recording meta information
    :return: speed limit
    """
    # Offset the markings by 1 because otherwise vehicles are driving on lane boundaries, there seems to be
    # an offset error somewhere in the commonroad conversion
    upper_lane_markings = [-float(x) + 1
                           for x in recording_df.upperLaneMarkings.values[0].split(";")]
    lower_lane_markings = [-float(x) + 1
                           for x in recording_df.lowerLaneMarkings.values[0].split(";")]
    len_upper = len(upper_lane_markings)
    len_lower = len(lower_lane_markings)
    upper_lane_markings[0] += extend_width
    upper_lane_markings[len_upper - 1] += -extend_width
    lower_lane_markings[0] += extend_width
    lower_lane_markings[len_lower - 1] += -extend_width
    return upper_lane_markings, lower_lane_markings


def get_dt(recording_df: DataFrame) -> float:
    """
    Extracts time step size from data frame

    :param recording_df: data frame of the recording meta information
    :return: time step size
    """
    return 1./recording_df.frameRate.values[0]


def get_speed_limit(recording_df: DataFrame) -> Union[float, None]:
    """
    Extracts speed limit from data frame

    :param recording_df: data frame of the recording meta information
    :return: speed limit
    """
    speed_limit = recording_df.speedLimit.values[0]
    if speed_limit < 0:
        return None
    else:
        return speed_limit


if __name__ == "__main__":
    # generate path to highd data files
    path_tracks = os.path.join(data_path, "*_tracks.csv")
    path_metas = os.path.join(data_path, "*_tracksMeta.csv")
    path_recording = os.path.join(data_path, "*_recordingMeta.csv")

    # get all file names
    listing_tracks = sorted(glob.glob(path_tracks))
    listing_metas = sorted(glob.glob(path_metas))
    listing_recording = sorted(glob.glob(path_recording))

    for index1, (recording_meta_fn, tracks_meta_fn, tracks_fn) in \
            enumerate(zip(listing_recording, listing_metas, listing_tracks)):

        tracks_meta_df = pd.read_csv(tracks_meta_fn, header=0)
        tracks_df = pd.read_csv(tracks_fn, header=0)
        recording_meta_df = pd.read_csv(recording_meta_fn, header=0)

        start_times, stop_times, ego_ids, to_remove_ids, merging_ids = find_valid_scenarios(
            tracks_meta_df, tracks_df, recording_meta_df)

        if not ego_ids:
            continue

        FPS = 1/get_dt(recording_meta_df)
        dt = get_dt(recording_meta_df)*DOWNSAMPLING
        speed_limit = get_speed_limit(recording_meta_df)
        upper_lane_markings, lower_lane_markings = get_lane_markings(
            recording_meta_df)

        meta_scenario = get_meta_scenario(
            dt, "MetaLower", lower_lane_markings, speed_limit, ROAD_LENGTH, Direction.LOWER, ROAD_OFFSET)

        if REMOVE_ALL_OTHER_TRAFFIC:
            output_dir = os.path.join(
                scenario_path, "highd_scenarios_no_traffic")
            os.makedirs(output_dir, exist_ok=True)
        else:
            output_dir = os.path.join(
                scenario_path, "highd_scenarios")
            os.makedirs(output_dir, exist_ok=True)

        for index2, (start, stop, ego, to_remove, merging) in enumerate(zip(start_times, stop_times, ego_ids, to_remove_ids, merging_ids)):
            scenario_id = "ZAM_{0}-{1}_{2}_T-1".format(
                "HighD", index1+1, index2+1)
            yaml_filepath = os.path.join(output_dir, scenario_id + ".yaml")
            generate_single_scenario(ego, to_remove, output_dir, tracks_df, tracks_meta_df,
                                     meta_scenario, scenario_id, Direction.LOWER, start, stop, True, DOWNSAMPLING)
            generate_yaml(recording_meta_df, tracks_df, tracks_meta_df,
                          ego, merging, start, stop, yaml_filepath)

            total_scenarios += 1

    print("Total scenarios: ", total_scenarios)
