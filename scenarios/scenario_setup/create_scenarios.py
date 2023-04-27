"""
IN ORDER TO USE, ENTER THIS FOLDER IN COMMAND LINE AND RUN:
    python3 create_scenarios.py ../../datasets/data ../

What we want to do:

Load the files from the HighD dataset, per location

Find scenarios with the following features:
- There is a lane-change (we need to detect when this happens)
- There is a car behind the changing car on the new lane. This will be the ego
- (?) The initial distance between ego and car in front of ego (if any) is according to the stand-still initialisation. This is because upon initialisation we have no info on velocity yet.
    - I actually would like to check how many scenarios would be disqualified due to this, otherwise maybe another way has to be found
- Find out if the merge is legal
    - Label otherwise as illegal cut-in

Convert the scenarios to XML (use cr map conversion? At least for lanes this would work well I guess). 
    - From tl before lane change to tu after lane change
    - Only use the lanes in the same direction.
    - Only add obstacles in the same direction.
    - Remove to-be-ego vehicle from dynamic obstacles

Create yaml file
    - Initial position, initial velocity
    - reference velocity is initial velocity
    - wether ego is truck or passenger vehicle
    - wether it is an illegal cut-in

Info such as speed limit and goal region can be put in main config yaml manually, which is per location.

FILE STRUCTURE
scenarios/
| highd_loc1/
| | highd_loc1_config.yaml
| | highd_loc1_1.xml
| | highd_loc1_1.yaml
| | highd_loc1_2.xml
| | highd_loc1_2.yaml
| highd_loc2/
| | highd_loc2_config.yaml
| | highd_loc2_1.xml
| | highd_loc2_1.yaml
| | highd_loc2_2.xml
| | highd_loc2_2.yaml
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

ROAD_LENGTH = 420  # For all scenarios
ROAD_OFFSET = 20  # To add before and after, avoiding traffic outside of lane
MAX_DEC = 4  # Maximum deceleration of ego to filter out any infeasible initial states

FPS = 25 # frames per second of the video
DESIRED_FREQ = 5 # desired simulation frequency
DOWNSAMPLING = int(FPS/DESIRED_FREQ) # highD framerate is 25 fps, we want to simulate at 5Hz

TIME_BEFORE = 3 # Time before a lane change that the scenario begins
TIME_AFTER = 5 #Time after a lane change that the scenario ends

total_scenarios = 0


def find_valid_scenarios(tracks_meta_df, tracks_df):

    def has_valid_initial_state(initial_state):
        global rejected_due_to_initial_state

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
        return first_frame, last_frame

    first_frames = []
    final_frames = []
    ego_ids = []
    remove_ids = []
    for vehicle_id in tracks_meta_df[tracks_meta_df.numLaneChanges == 1].id:
        final_frame = tracks_meta_df[tracks_meta_df.id ==
                                     vehicle_id].finalFrame.values[0]
        ego_id = tracks_df[(tracks_df.id == vehicle_id) & (
            tracks_df.frame == final_frame)].followingId.values
        
        # Check that there was a car behind (which will be the ego vehicle)
        if not ego_id:
            continue
        ego_id = ego_id[0]

        # Check that ego vehicle exists over the whole time interval
        first_frame, final_frame = find_lane_change_interval(tracks_df[tracks_df.id == vehicle_id])
        if first_frame < 0:
            continue
        if len(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)].index) == 0:
            continue
        if len(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == final_frame)].index) == 0:
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

        # Check that the merging happens close enough to the vehicle, otherwise it is not really a cut-in
        if tracks_df[tracks_df.id == ego_id].dhw.min() > 120:
            continue

        remove_id = tracks_df[(tracks_df.id == ego_id) & (
            tracks_df.frame == first_frame)].followingId.values

        first_frames.append(first_frame)
        final_frames.append(final_frame)
        ego_ids.append(ego_id)
        remove_ids.append(remove_id)

    return first_frames, final_frames, ego_ids, remove_ids


def generate_yaml(video_meta_df, tracks_df, tracks_meta_df, ego_id, first_frame, final_frame, downsampling, output_filename):
    if video_meta_df["speedLimit"].values[0] == -1:
        speed_limit = 50
    else:
        speed_limit = 1.2*video_meta_df["speedLimit"].values[0]

    data = dict(
        simulation_duration = int((final_frame - first_frame) // downsampling),
        
        initial_state_x = float(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["x"].values[0]),
        initial_state_y = float(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["y"].values[0]),
        initial_state_orientation = 0,
        initial_state_velocity = float(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["xVelocity"].values[0]),
        
        reference_velocity = float(tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["xVelocity"].values[0]),
        vmax = int(speed_limit),

        planning_horizon = int(max((tracks_df[(tracks_df.id == ego_id) & (tracks_df.frame == first_frame)]["xVelocity"].values[0] + 4.99) // 5 * 5, 30)),

        vehicle_type = tracks_meta_df[tracks_meta_df.id == ego_id]["class"].values[0],
        vehicle_length = float(tracks_meta_df[tracks_meta_df.id == ego_id]["width"].values[0]),
        vehicle_width = float(tracks_meta_df[tracks_meta_df.id == ego_id]["height"].values[0])
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
    upper_lane_markings = [-float(x)
                           for x in recording_df.upperLaneMarkings.values[0].split(";")]
    lower_lane_markings = [-float(x)
                           for x in recording_df.lowerLaneMarkings.values[0].split(";")]
    len_upper = len(upper_lane_markings)
    len_lower = len(lower_lane_markings)
    # -8 + 1 = -7
    upper_lane_markings[0] += extend_width
    # -16 -1 = -17
    upper_lane_markings[len_upper - 1] += -extend_width
    # -22 + 1 = -21
    lower_lane_markings[0] += extend_width
    # -30 -1 = -31
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

        start_times, stop_times, ego_ids, to_remove_ids = find_valid_scenarios(
            tracks_meta_df, tracks_df)

        if not ego_ids:
            continue

        recording_meta_df = pd.read_csv(recording_meta_fn, header=0)
        FPS = 1/get_dt(recording_meta_df)
        dt = get_dt(recording_meta_df)*DOWNSAMPLING
        speed_limit = get_speed_limit(recording_meta_df)
        upper_lane_markings, lower_lane_markings = get_lane_markings(
            recording_meta_df)

        meta_scenario = get_meta_scenario(
            dt, "MetaLower", upper_lane_markings, speed_limit, ROAD_LENGTH, Direction.LOWER, ROAD_OFFSET)

        output_dir = os.path.join(
            scenario_path, "highd_loc{0}".format(index1+1))
        os.makedirs(output_dir, exist_ok=True)

        for index2, (start, stop, ego, to_remove) in enumerate(zip(start_times, stop_times, ego_ids, to_remove_ids)):
            scenario_id = "ZAM_{0}-{1}_{2}_T-1".format("HighD", index1+1, index2+1)
            yaml_filepath = os.path.join(output_dir, scenario_id + ".yaml")
            generate_single_scenario(ego, to_remove, output_dir, tracks_df, tracks_meta_df,
                                     meta_scenario, scenario_id, Direction.LOWER, start, stop, True, DOWNSAMPLING)
            generate_yaml(recording_meta_df, tracks_df, tracks_meta_df, ego, start, stop, DOWNSAMPLING, yaml_filepath)

            total_scenarios += 1

    print("Total scenarios: ", total_scenarios)