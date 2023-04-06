import numpy as np
from dataclasses import dataclass

from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import MultiPolygon as ShapelyMultiPolygon
from shapely.geometry import GeometryCollection as ShapelyGeometryCollection
from shapely.geometry import MultiPoint
from commonroad.geometry.shape import Polygon as CommonRoadPolygon
from commonroad.scenario.scenario import Lanelet

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
    # - If point 3 is on the same line as between point 1 and 2, remove point 2
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
    right_bound_length = 0
    for vertex in right[1:]:
        right_bound_length += np.sqrt((vertex[0] - previous_vertex[0])**2 + (
            vertex[1] - previous_vertex[1])**2)
        previous_vertex = vertex

    previous_vertex = left[0]
    left_bound_length = 0
    for vertex in left[1:]:
        left_bound_length += np.sqrt((vertex[0] - previous_vertex[0])**2 + (
            vertex[1] - previous_vertex[1])**2)
        previous_vertex = vertex

    right_scaling = np.sqrt(lane_length/right_bound_length)
    left_scaling = np.sqrt(lane_length/left_bound_length)
    # For the width, average between width beginning point and end point
    lane_width = 0.5*np.sqrt((right[-1][0]-left[0][0])**2 + (right[-1][1]-left[0][1])**2) + 0.5*np.sqrt(
        (right[0][0]-left[-1][0])**2 + (right[0][1]-left[-1][1])**2)

    mapped_right = [[0, 0]]
    for previous_ind, vertex in enumerate(right[1:]):
        distance = np.sqrt(
            (vertex[0]-right[previous_ind][0])**2 + (vertex[1]-right[previous_ind][1])**2)
        mapped_right.append([mapped_right[-1][0]+distance*right_scaling, 0])

    mapped_left = [[lane_length, lane_width]]
    for previous_ind, vertex in enumerate(left[1:]):
        distance = np.sqrt(
            (vertex[0]-left[previous_ind][0])**2 + (vertex[1]-left[previous_ind][1])**2)
        mapped_left.append(
            [mapped_left[-1][0]-distance*left_scaling, lane_width])

    original_lane_shape = np.concatenate((right, left))
    original_lane = ShapelyPolygon(original_lane_shape)
    assert original_lane.is_valid, "Lane does not have a valid shape"

    mapped_lane_shape = np.concatenate((mapped_right, mapped_left))
    mapped_lane = ShapelyPolygon(mapped_lane_shape)
    assert mapped_lane.is_valid, "Mapped lane does not have a valid shape"

    return original_lane, mapped_lane


def plot_polygon(shapely_polygon):
    plt.plot(*shapely_polygon.exterior.xy)
    for i in range(len(shapely_polygon.exterior.xy[0])):
        plt.text(shapely_polygon.exterior.xy[0][i],
                 shapely_polygon.exterior.xy[1][i], str(i))
    plt.show()
