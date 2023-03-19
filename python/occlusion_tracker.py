#from CGAL.CGAL_Polyhedron_3 import Polyhedron_3 as Polyhedron
#from CGAL.CGAL_Polygon_mesh_processing import do_intersect
#from commonroad.scenario.scenario import Lanelet
#from utilities import LaneletToCGALPolygon, ReachabilityBounds
from py_occlusions import ReachabilityParams, OcclusionHandler
from shapely.geometry import Polygon

point1 = [0, 1]
point2 = [1, 1]
point3 = [1, 0]
point4 = [0, 0]
poly1 = Polygon([point1, point2, point3])
poly2 = Polygon([point1, point3, point4])

polylist = [poly1, poly2]

params = ReachabilityParams()

occtest = OcclusionHandler(polylist, poly1, 0, params)

occtest.update(poly2)

output = occtest.get_reachable_sets()
print(output)

print("Wow, we didn't get any errors")

"""
class Shadow:
    #polyhedron: Polyhedron
    #driving_corridor: Lanelet
    #driving_corridor_extruded: Polyhedron

    def __init__(self, polyhedron, driving_corridor):
        self.polyhedron = polyhedron
        self.driving_corridor = driving_corridor
        driving_corridor_CGAL = LaneletToCGALPolygon(driving_corridor)
        # Extrude driving corridor to [0, v_max]

    def expand(self, dt, polyhedron = None):
        
        #This function expands the shadow polyhedron using reachability
        
        if not polyhedron: polyhedron = self.polyhedron

        # Note that this proposed expansion algorithm is an over-approximation. The proper
        # reachable set computations are more advanced (maybe see later if it is possible)
        #
        # a: skew along x with each vertex's Z-coord (which is vx)*dt + 1/2*a_max*dt^2
        # b: skew along x with each vertex's Z-coord (which is vx)*dt - 1/2*a_min*dt^2
        # Take union of a & b
        # Extrude in Z-dir by a_max*dt on top and a_min*dt on bottom
        # Extrude in Y-dit by y_vel*dt in both directions
        #
        # Take intersection between expanded poly and extruded driving corridor
        #
        # Maybe the extrusions can be done by taking a minkowsky sum with a vector

    def get_cr_occupancy_set(self, time_step, dt, prediction_horizon):
        
        #This function performs the reachability of the shadow over a prediction horizon and
        #returns the occupancy set for a dynamic obstacle in CommonRoad
        
        # polyhedron = self.polyhedron # Make a copy
        # occupancy_set = []
        # for i in range(prediction_horizon):
        #   expand(dt, polyhedron)
        #   projected = xy_project(polyhedron)
        #   occupancy_set.append(projected)

        pass

class OcclusionTracker:
    time_step: int

    def __init__(self, scenario, sensor_view, initial_time_step=0, prediction_horizon=10):
        # Find all driving corridors

        # Calculate the first view:
        # For each driving corridor:
        #   Take differences between driving corridor and sensorview
        #   Extrude differences to full range of longitudinal vel
        #   Append all separate polyhedrons to shadow list
        pass

    def update(self, sensor_view, new_time_step):
        
        #This function updates the occlusion polyhedrons for the whole scene
        
        # t = new_time_step - self.time_step
        # self.time_step = new_time_step
        # 
        # Extrude sensor_view to full range of longitudinal vel
        # 
        # For each shadow in list:
        #   shadow.expand(t)
        #   new_shadow_list.append = Boolean operation shadow and sensor_view (Note this could give multiple new polyhedra as they could have splitted)
        # shadow_list = new_shadow_list

        pass

    def get_cr_dynamic_obstacles(self, scenario):
        
        #This function returns the shadows as dynamic obstacles to be used in CommonRoad
        
        # For each shadow in shadowlist:
        #   occupancy_set = shadow.get_cr_occupancy_set(time_step, dt, prediction_horizon)
        #   Use this occupancy set to set up dynamic obstacle
        pass
"""