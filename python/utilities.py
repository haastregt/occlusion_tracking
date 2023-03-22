import numpy as np
from dataclasses import dataclass

from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import MultiPolygon as ShapelyMultiPolygon
from shapely.geometry import GeometryCollection as ShapelyGeometryCollection
from shapely.geometry import MultiPoint
from commonroad.geometry.shape import Polygon as CommonRoadPolygon
from commonroad.scenario.scenario import Lanelet

def ShapelyPolygon2Polygon(shapely_polygon):
    assert isinstance(shapely_polygon, ShapelyPolygon)
    assert hasattr(shapely_polygon, 'exterior')
    assert hasattr(shapely_polygon.exterior, 'xy')
    vertices = np.array(list(zip(*shapely_polygon.exterior.xy)))
    return CommonRoadPolygon(vertices)

def rgb2hex(r,g,b):
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