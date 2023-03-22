from occlusion_tracker import OcclusionTracker
from commonroad.scenario.scenario import Scenario
from shapely.geometry import Polygon

point1 = [0, 25]
point2 = [25, 25]
point3 = [25, 0]
point4 = [0, 0]
point5 = [10, 10]
poly1 = Polygon([point4, point3, point5, point2, point1])
poly2 = Polygon([point1, point3, point4])

polylist = [poly1, poly2]
scenario = Scenario(dt=0.1)
tracker = OcclusionTracker(scenario,poly1)

tracker.update(poly1, 1)

#output = tracker.get_cr_dynamic_obstacles()
#print(output)

print("Wow, we didn't get any errors")