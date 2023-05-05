import numpy as np
import pickle
import os
import glob
import tqdm
import pandas as pd
from dataclasses import dataclass

from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import MultiPolygon as ShapelyMultiPolygon
from shapely.geometry import GeometryCollection as ShapelyGeometryCollection
from shapely.geometry import MultiPoint, Point
from commonroad.geometry.shape import Polygon as CommonRoadPolygon
from commonroad.scenario.scenario import Lanelet, LaneletNetwork

from shapely.geometry import LineString
import matplotlib.pyplot as plt


def ShapelyPolygon2Polygon(shapely_polygon):
    assert isinstance(shapely_polygon, ShapelyPolygon)
    assert hasattr(shapely_polygon, 'exterior')
    assert hasattr(shapely_polygon.exterior, 'xy')
    vertices = np.array(list(zip(*shapely_polygon.exterior.xy)))
    return CommonRoadPolygon(vertices)


def rgb2hex(r, g, b):
    def clamp(x):
        return max(0, min(x, 255))

    return "#{0:02x}{1:02x}{2:02x}".format(clamp(r), clamp(g), clamp(b))


def polygon_union(polygons):
    current_polygon = ShapelyPolygon()
    for polygon in polygons:
        current_polygon = current_polygon.union(polygon)
    polygon_list = filter_polygons(current_polygon)
    return polygon_list


def filter_polygons(input):
    polygonEmpty = ShapelyPolygon()
    polygon_list = []
    if isinstance(input, ShapelyPolygon):
        if input != polygonEmpty:
            assert input.is_valid
            polygon_list.append(input)
    elif isinstance(input, ShapelyMultiPolygon):
        for polygon in input.geoms:
            if polygon != polygonEmpty:
                assert polygon.is_valid
                polygon_list.append(polygon)
    elif isinstance(input, ShapelyGeometryCollection):
        for element in input.geoms:
            if isinstance(element, ShapelyPolygon):
                if element != polygonEmpty:
                    assert element.is_valid
                    polygon_list.append(element)
    return polygon_list


def Lanelet2ShapelyPolygon(lanelet):
    assert isinstance(lanelet, Lanelet)
    right = lanelet.right_vertices
    left = np.flip(lanelet.left_vertices, axis=0)
    lanelet_boundary = np.concatenate((right, left, np.array([right[0]])))

    lanelet_shapely = ShapelyPolygon(lanelet_boundary)
    if not lanelet_shapely.is_valid:
        lanelet_shapely = lanelet_shapely.buffer(0)
        if not lanelet_shapely.is_valid:
            print("Note: Shape of lanelet", lanelet.lanelet_id,
                  "is not valid, creating valid shape with convex hull of lane boundary.")
            lanelet_shapely = MultiPoint(lanelet_boundary).convex_hull
            assert lanelet_shapely.is_valid, "Failed to convert lanelet to polygon"
    return lanelet_shapely


def ShapelyRemoveDoublePoints(polygon, tolerance):
    x_prev = 99999999999999
    y_prev = 99999999999999

    new_coords = []
    for coord in polygon.exterior.coords[:-1]:
        x = coord[0]
        y = coord[1]
        if abs(x-x_prev) < tolerance and abs(y - y_prev) < tolerance:
            continue
        else:
            new_coords.append(coord)
            x_prev = x
            y_prev = y

    x, y = polygon.exterior.coords[-1]
    if abs(x-x_prev) < tolerance and abs(y - y_prev) < tolerance:
        new_coords = new_coords[:-1]

    return ShapelyPolygon(new_coords)


def process_vertices(points, tolerance):
    # The processing does the following to a sequence of points:
    # - Remove double points
    # - If point 3 is on the line passing through point 1 and 2, remove point 2
    new_points = [points[0]]
    for point in points[1:]:
        if not (point[0]-new_points[-1][0])**2 + (point[1]-new_points[-1][1])**2 > tolerance**2:
            continue

        if len(new_points) < 2:
            new_points.append(point)
            continue

        crossproduct = (point[1] - new_points[-2][1]) * (new_points[-1][0] - new_points[-2][0]) - (
            point[0] - new_points[-2][0]) * (new_points[-1][1] - new_points[-2][1])
        normalization = np.linalg.norm([point[0]-new_points[-2][0], point[1]-new_points[-2][1]])*np.linalg.norm([
            new_points[-1][0]-new_points[-2][0], new_points[-1][1]-new_points[-2][1]])
        if abs(crossproduct/normalization) > 0.001:
            new_points.append(point)
            continue
        new_points[-1] = point

    return np.array(new_points)


def create_lane_shapes(lane):
    right = process_vertices(lane.right_vertices, 0.1)
    left = process_vertices(np.flip(lane.left_vertices, axis=0), 0.1)
    center = process_vertices(lane.center_vertices, 0.1)

    previous_vertex = center[0]
    lane_length = 0
    for vertex in center[1:]:
        lane_length += np.sqrt((vertex[0] - previous_vertex[0])
                               ** 2 + (vertex[1] - previous_vertex[1])**2)
        previous_vertex = vertex

    previous_vertex = right[0]
    right_edges = np.zeros((len(right)-1,))
    right_factors = np.zeros((len(right),))
    for ind, vertex in enumerate(right[1:]):
        right_edges[ind] = np.linalg.norm(vertex-previous_vertex)
        if ind < len(right) - 2:
            previous_edge = vertex - previous_vertex 
            next_edge = right[ind + 2] - vertex
            angle = np.arccos(np.dot(previous_edge,next_edge)/(np.linalg.norm(previous_edge)*np.linalg.norm(next_edge)))
            right_factors[ind + 1] = angle/np.max([np.linalg.norm(previous_edge), np.linalg.norm(next_edge)])

        previous_vertex = vertex

    right_bound_length = np.sum(right_edges)
    right_factors = right_factors/np.sum(right_factors)

    previous_vertex = left[0]
    left_edges = np.zeros((len(left)-1,))
    left_factors = np.zeros((len(left),))
    for ind, vertex in enumerate(left[1:]):
        left_edges[ind] = np.linalg.norm(vertex-previous_vertex)
        if ind < len(left) - 2:
            previous_edge = vertex - previous_vertex 
            next_edge = left[ind + 2] - vertex
            angle = np.arccos(np.dot(previous_edge,next_edge)/(np.linalg.norm(previous_edge)*np.linalg.norm(next_edge)))
            left_factors[ind + 1] = angle/np.max([np.linalg.norm(previous_edge), np.linalg.norm(next_edge)])

        previous_vertex = vertex

    left_bound_length = np.sum(left_edges)
    left_factors = left_factors/np.sum(left_factors)

    lane_width = 0.5*np.sqrt((right[-1][0]-left[0][0])**2 + (right[-1][1]-left[0][1])**2) + 0.5*np.sqrt(
        (right[0][0]-left[-1][0])**2 + (right[0][1]-left[-1][1])**2)

    left_difference = lane_length - left_bound_length
    right_difference = lane_length - right_bound_length

    mapped_right = [[0, 0]]
    for previous_ind, vertex in enumerate(right[1:]):
        distance = np.sqrt(
            (vertex[0]-right[previous_ind][0])**2 + (vertex[1]-right[previous_ind][1])**2)
        mapped_right.append([mapped_right[-1][0]+distance + right_difference*right_factors[previous_ind+1], 0])

    mapped_left = [[lane_length, lane_width]]
    for previous_ind, vertex in enumerate(left[1:]):
        distance = np.sqrt(
            (vertex[0]-left[previous_ind][0])**2 + (vertex[1]-left[previous_ind][1])**2)   
        mapped_left.append(
            [mapped_left[-1][0]-distance - left_difference*left_factors[previous_ind + 1], lane_width])

    original_lane_shape = np.concatenate((right, left))
    original_lane = ShapelyPolygon(original_lane_shape)
    assert original_lane.is_valid, "Lane does not have a valid shape"

    mapped_lane_shape = np.concatenate((mapped_right, mapped_left))
    mapped_lane = ShapelyPolygon(mapped_lane_shape)
    assert mapped_lane.is_valid, "Mapped lane does not have a valid shape"

    assert len(original_lane.exterior.coords[:]) == len(
        mapped_lane.exterior.coords[:]), "Number of vertices for original and mapped polygons have to be the same"

    return original_lane, mapped_lane

def create_dc_shapes(lanelet_network: LaneletNetwork):
    # Find the leftmost initial lanelets
    initial_lanelets = []
    initial_lanelets_and_neighbours = []
    for lanelet in lanelet_network.lanelets:
        if lanelet.predecessor:
            continue
        if lanelet.lanelet_id in initial_lanelets_and_neighbours:
            continue
        initial_lanelets_and_neighbours.append(lanelet.lanelet_id)

        current = lanelet
        while current.adj_left_same_direction:
            current = lanelet_network.find_lanelet_by_id(current.adj_left)
            initial_lanelets_and_neighbours.append(current.lanelet_id)

        initial_lanelets.append(current)
        
        current = lanelet
        while current.adj_right_same_direction:
            current = lanelet_network.find_lanelet_by_id(current.adj_right)
            initial_lanelets_and_neighbours.append(current.lanelet_id)

    # Find driving corridors, which consist of a sequence of parallel slices of lanelets from leftmost to rightmost
    driving_corridors = []
    for lanelet in initial_lanelets:
        stack = [[[], lanelet]] #Stack of [Unfinised driving corridors, successor list at that point]
        
        while stack:
            current_from_stack = stack.pop(0)
            driving_corridor = [current_from_stack[0]] if current_from_stack[0] else []

            previous_successors = [current_from_stack[1].lanelet_id]

            while True:
                leftmost_lanelet = lanelet_network.find_lanelet_by_id(previous_successors.pop(0))
                parallel_slice = [leftmost_lanelet]

                current = leftmost_lanelet
                if current.successor:
                    successors_list = current.successor
                    # Make sure next parallel slice will have leftmost as first again
                    while lanelet_network.find_lanelet_by_id(successors_list[0]).adj_left_same_direction:
                        successors_list[0] = lanelet_network.find_lanelet_by_id(successors_list[0]).adj_left
                else:
                    successors_list = []
                    
                while current.adj_right_same_direction:
                    current = lanelet_network.find_lanelet_by_id(current.adj_right)
                    parallel_slice.append(current)
                    if current.lanelet_id in previous_successors:
                        previous_successors.remove(current.lanelet_id)
                    if current.successor:
                        successors_list.extend(current.successor)

                driving_corridor.append(parallel_slice)

                if previous_successors:
                    #Then dc has split, duplicate current driving corridor state and the remaining previous successors
                    #to a stack to find later
                    stack.append([driving_corridor, previous_successors])
                    pass

                if not successors_list:
                    break

                previous_successors = successors_list

                
            driving_corridors.append(driving_corridor)

    # Now find the list of lanes in each driving corridor
    lanes_in_dc = []
    for dc in driving_corridors:
        initial_lanelets = dc[0]
        this_dc_lanes = []
        for lanelet in initial_lanelets:
            lanes, _ = Lanelet.all_lanelets_by_merging_successors_from_lanelet(
                    lanelet, lanelet_network, max_length=500)
            for lane in lanes:
                this_dc_lanes.append(Lanelet2ShapelyPolygon(lane))
        lanes_in_dc.append(this_dc_lanes)

    # Now that the driving corridors have been found, they can be processed to get the shapes
    original_lanes = []
    mapped_lanes = []
    for dc in driving_corridors:
        left_original = [dc[0][0].left_vertices[0]]
        right_original = [dc[0][-1].right_vertices[0]]
        
        # Keep track of end of previous end-vertices
        prev_left = dc[0][0].left_vertices[0]
        prev_right = dc[0][-1].right_vertices[0]
        # Width for mapped version, compute first width prior to loop
        y_min = 0
        y_max = np.linalg.norm(dc[0][0].left_vertices[0] - dc[0][-1].right_vertices[0])
        xbegin = 0
        left_mapped = [[xbegin, y_max]]
        right_mapped = [[xbegin, y_min]]
        for parallel_slice in dc:
            left = process_vertices(parallel_slice[0].left_vertices, 0.1)
            right = process_vertices(parallel_slice[-1].right_vertices, 0.1)

            change_left = np.linalg.norm(prev_left - left[0])
            v1 = left[1] - left[0]
            v2 = prev_left - left[0]
            direction = np.arctan2(v1[0]*v2[1]-v1[1]*v2[0], v1[0]*v2[0]+v1[1]*v2[1])
            if change_left < 0.01:
                left_original.extend(left[1:])
            else:
                left_original.extend(left)
                y_max -= np.sign(direction)*change_left

            change_right = np.linalg.norm(prev_right - right[0])
            v1 = right[1] - right[0]
            v2 = prev_right - right[0]
            direction = np.arctan2(v1[0]*v2[1]-v1[1]*v2[0], v1[0]*v2[0]+v1[1]*v2[1])
            if change_right < 0.01:
                right_original.extend(right[1:])
            else:
                right_original.extend(right)
                y_min -= np.sign(direction)*change_right
            
            left_bound_mapped, right_bound_mapped = create_mapped_bounds(left, right, y_min, y_max, xbegin)
            xbegin = left_bound_mapped[-1][0]
            if change_left < 0.01:
                left_mapped.extend(left_bound_mapped[1:])
            else:
                left_mapped.extend(left_bound_mapped)
            if change_right < 0.01:
                right_mapped.extend(right_bound_mapped[1:])
            else:
                right_mapped.extend(right_bound_mapped)

            prev_left = left[-1]
            prev_right = right[-1]

        left_original.reverse()
        original_lane_shape = np.concatenate((right_original, left_original))
        original_lane = ShapelyPolygon(original_lane_shape)
        original_lanes.append(original_lane)

        left_mapped.reverse()
        mapped_lane_shape = np.concatenate((right_mapped, left_mapped))
        mapped_lane = ShapelyPolygon(mapped_lane_shape)
        mapped_lanes.append(mapped_lane)

        assert len(original_lane.exterior.coords[:]) == len(
        mapped_lane.exterior.coords[:]), "Number of vertices for original and mapped polygons have to be the same"
    
    return original_lanes, mapped_lanes, lanes_in_dc

def create_mapped_bounds(left, right, ymin, ymax, xbegin):
    previous_vertex = right[0]
    right_edges = np.zeros((len(right)-1,))
    right_factors = np.zeros((len(right),))
    for ind, vertex in enumerate(right[1:]):
        right_edges[ind] = np.linalg.norm(vertex-previous_vertex)
        if ind < len(right) - 2:
            previous_edge = vertex - previous_vertex 
            next_edge = right[ind + 2] - vertex
            angle = np.arccos(np.dot(previous_edge,next_edge)/(np.linalg.norm(previous_edge)*np.linalg.norm(next_edge)))
            right_factors[ind + 1] = angle/np.max([np.linalg.norm(previous_edge), np.linalg.norm(next_edge)])

        previous_vertex = vertex

    right_bound_length = np.sum(right_edges)
    if np.sum(right_factors) == 0: #This happens if there are only two vertices
        right_factors = np.ones((len(right),))/len(right)
    else:
        right_factors = right_factors/np.sum(right_factors)

    previous_vertex = left[0]
    left_edges = np.zeros((len(left)-1,))
    left_factors = np.zeros((len(left),))
    for ind, vertex in enumerate(left[1:]):
        left_edges[ind] = np.linalg.norm(vertex-previous_vertex)
        if ind < len(left) - 2:
            previous_edge = vertex - previous_vertex 
            next_edge = left[ind + 2] - vertex
            angle = np.arccos(np.dot(previous_edge,next_edge)/(np.linalg.norm(previous_edge)*np.linalg.norm(next_edge)))
            left_factors[ind + 1] = angle/np.max([np.linalg.norm(previous_edge), np.linalg.norm(next_edge)])

        previous_vertex = vertex

    left_bound_length = np.sum(left_edges)
    if np.sum(left_factors) == 0: #This happens if there are only two vertices
        left_factors = np.ones((len(left),))/len(left)
    else:
        left_factors = left_factors/np.sum(left_factors)

    lane_length = 0.5*(left_bound_length + right_bound_length)
    left_difference = lane_length - left_bound_length
    right_difference = lane_length - right_bound_length

    mapped_right = [[xbegin, ymin]]
    for previous_ind, vertex in enumerate(right[1:]):
        distance = np.sqrt(
            (vertex[0]-right[previous_ind][0])**2 + (vertex[1]-right[previous_ind][1])**2)
        mapped_right.append([mapped_right[-1][0]+distance + right_difference*right_factors[previous_ind+1], ymin])

    mapped_left = [[xbegin, ymax]]
    for previous_ind, vertex in enumerate(left[1:]):
        distance = np.sqrt(
            (vertex[0]-left[previous_ind][0])**2 + (vertex[1]-left[previous_ind][1])**2)   
        mapped_left.append(
            [mapped_left[-1][0]+distance + left_difference*left_factors[previous_ind + 1], ymax])

    return mapped_left, mapped_right

def plot_polygon(shapely_polygon):
    plt.plot(*shapely_polygon.exterior.xy)
    for i in range(len(shapely_polygon.exterior.xy[0])):
        plt.text(shapely_polygon.exterior.xy[0][i],
                 shapely_polygon.exterior.xy[1][i], str(i))
    plt.show()

def find_RSS_distance(ego_vehicle, scenario, config):
    # First, we need to find the lane from ego vehicle to goal location
    initial_state = [config.get('initial_state_x'), config.get('initial_state_y')]
    goal_point = [config.get('goal_point_x'), config.get('goal_point_y')]

    starting_lanelet_ids = scenario.lanelet_network.find_lanelet_by_position(
        [initial_state])[0]
    
    starting_lanelets = []
    for lanelet_id in starting_lanelet_ids:
        starting_lanelets.append(
            scenario.lanelet_network.find_lanelet_by_id(lanelet_id))
        
    ego_lane = []
    for lanelet in starting_lanelets:
        starting_lanes = lanelet.all_lanelets_by_merging_successors_from_lanelet(
            lanelet, scenario.lanelet_network)[0]
        for lane in starting_lanes:
            lane_shape = Lanelet2ShapelyPolygon(lane)
            if lane_shape.intersects(Point(*goal_point)):
                ego_lane = lane_shape
                break
        else:
            continue
        break

    # Then for each timestep, find all objects that are on this lane AND in front of the ego vehicle
    # Compute distances to these objects and lowest distance is RSS distance
    RSS = []
    for t in range(1,config.get('simulation_duration')):
        distances = []
        for traffic in scenario.obstacle_states_at_time_step(t).values():
            traffic_pos = traffic.position
            if not ego_lane.intersects(Point(*traffic_pos)):
                continue

            ego_pos = ego_vehicle.prediction.trajectory.state_list[t].position
            angle = np.arccos(np.dot(ego_pos,traffic_pos)/(np.linalg.norm(ego_pos)*np.linalg.norm(traffic_pos)))
            diff = traffic_pos - ego_pos
            distance = np.linalg.norm(diff)
            if not np.abs(angle) < np.pi/2:
                continue
            distances.append(distance)

        if distances:
            RSS.append(min(distances))
        else:
            RSS.append(0)

    return RSS

def merge_config(global_config, scenario_config):
    global_config["simulation_duration"]                    = scenario_config["simulation_duration"]
    global_config["initial_state_x"]                        = scenario_config["initial_state_x"]
    global_config["initial_state_y"]                        = scenario_config["initial_state_y"]
    global_config["initial_state_orientation"]              = scenario_config["initial_state_orientation"]
    global_config["initial_state_velocity"]                 = scenario_config["initial_state_velocity"]
    global_config["vehicle_type"]                           = scenario_config["vehicle_type"]
    global_config["vehicle_length"]                         = scenario_config["vehicle_length"]
    global_config["vehicle_width"]                          = scenario_config["vehicle_width"]
    global_config["reference_speed"]                        = scenario_config["reference_velocity"]
    global_config["planning_horizon"]                       = scenario_config["planning_horizon"]
    global_config["goal_point_x"]                           = scenario_config["goal_point_x"]
    global_config["goal_point_y"]                           = scenario_config["goal_point_y"]
    global_config["occlusion_params"]["vmax"]               = scenario_config["vmax"]
    global_config["occlusion_params"]["prediction_horizon"] = scenario_config["planning_horizon"]

    return global_config

def save_results(file_path, tracked_results, untracked_results, scenario, scenario_config):
    scenario_name = str(scenario.scenario_id)
    data = {
        "simulation_length" : scenario_config["simulation_duration"],
        "scenario"          : scenario,
        "scenario_name"     : scenario_name,
        "novel_method"      : {"ego_vehicle"        : tracked_results[0],
                               "scenarios"          : tracked_results[1],
                               "views"              : tracked_results[2],
                               "shadows"            : tracked_results[3],
                               "emergency_brakes"   : tracked_results[4],
                               "computational_time" : {"update_step"    : tracked_results[5][0],
                                                       "prediction_step": tracked_results[5][1]}},
        "baseline_method"   : {"ego_vehicle"        : untracked_results[0],
                               "scenarios"          : untracked_results[1],
                               "views"              : untracked_results[2],
                               "shadows"            : untracked_results[3],
                               "emergency_brakes"   : untracked_results[4],
                               "computational_time" : {"update_step"    : untracked_results[5][0],
                                                       "prediction_step": untracked_results[5][1]}},
        "vehicle_type"      : scenario_config["vehicle_type"], 
        "ego_speed"         : scenario_config["reference_velocity"],
        "recorded_ego_speed": scenario_config["recorded_ego_velocity"],
        "other_speed"       : scenario_config["v_other"],
        "legal_merge"       : scenario_config["legal_merge"],
        "is_overtake"       : scenario_config["is_overtake"],
        "merge_ttc"         : scenario_config["merge_ttc"],
        "merge_dhw"         : scenario_config["merge_dhw"]
    }
    file = open(file_path, 'wb')
    pickle.dump(data, file)
    file.close()

def load_results(file_path):
    file = open(file_path, 'rb')
    data = pickle.load(file)
    file.close()
    
    return data

def single_to_batch_results(single_results_folder, batch_results_path):
    TIMESTEP_CUTIN = int(3.6/0.2) # Used to find average velocities after cut-in has happened
    
    single_result_paths = os.path.join(single_results_folder, "*")
    result_list = glob.glob(single_result_paths)

    dict_list = []
    for result in tqdm.tqdm(result_list):
        scenario_id = os.path.basename(result)
        data = load_results(result)

        avg_vel_recorded = np.average(np.clip(data["recorded_ego_speed"][TIMESTEP_CUTIN-1:], a_min = 0, a_max = data["ego_speed"]))
        avg_vel_novel = np.average([state.velocity for state in data["novel_method"]["ego_vehicle"].prediction.trajectory.state_list][TIMESTEP_CUTIN-1:])
        avg_vel_baseline = np.average([state.velocity for state in data["baseline_method"]["ego_vehicle"].prediction.trajectory.state_list][TIMESTEP_CUTIN-1:])

        n_brakes_novel = np.sum(data["novel_method"]["emergency_brakes"])
        n_brakes_baseline = np.sum(data["baseline_method"]["emergency_brakes"])

        entry_dict = {
            "scenario_id"       : scenario_id,

            "avg_vel_novel"     : avg_vel_novel,
            "avg_vel_baseline"  : avg_vel_baseline,
            "avg_vel_recorded"  : avg_vel_recorded,
            
            "n_brakes_novel"    : n_brakes_novel,
            "n_brakes_baseline" : n_brakes_baseline,
            
            "comp_time_update"  : data["novel_method"]["computational_time"]["update_step"],
            "comp_time_predict" : data["novel_method"]["computational_time"]["prediction_step"],

            "vehicle_type"      : data["vehicle_type"], 
            "ego_speed"         : data["ego_speed"],
            "other_speed"       : data["other_speed"],
            "legal_merge"       : data["legal_merge"],
            "is_overtake"       : data["is_overtake"],
            "merge_ttc"         : data["merge_ttc"],
            "merge_dhw"         : data["merge_dhw"]
        }

        dict_list.append(entry_dict)

    df = pd.DataFrame(dict_list)
    df.to_pickle(batch_results_path)