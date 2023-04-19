import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from commonroad.scenario.obstacle import ObstacleType
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.visualization.draw_params import MPDrawParams, DynamicObstacleParams, ShapeParams
from utilities import ShapelyPolygon2Polygon, rgb2hex


class Visualizer:
    def __init__(self):
        self.animation = None
        self.fig = None

    def _update(self, time_step, scenarios, sensor_views, ego_id):
        plt.clf()
        scenario = scenarios[time_step]
        ego_vehicle = scenario.obstacle_by_id(ego_id)
        self.plot(scenario=scenario,
                  ego_vehicle=ego_vehicle,
                  sensor_view=sensor_views[time_step],
                  time_begin=time_step)
        plt.autoscale()
        plt.axis('equal')
        plt.xlim(0, 100)
        plt.ylim(-50, 50)

    def draw_shadows(self, rnd, shadows, time_begin, time_horizon):
        # We use red
        R = int(255)
        G = int(0)
        B = int(0)

        # Draw the first location of the shadows
        draw_params = DynamicObstacleParams.load(
            file_path="python/draw_params/shadow.yaml", validate_types=False)
        draw_params.time_begin = time_begin
        draw_params.time_end = time_begin + 1
        for shadow in shadows:
            shadow.draw(rnd, draw_params=draw_params)

        # Draw the shadow predictions
        draw_params = DynamicObstacleParams.load(
            file_path="python/draw_params/shadow_prediction.yaml", validate_types=False)
        for i in reversed(range(time_horizon)):
            tint_factor = 0.005**(1/(i+1))
            Ri = int(R + (255 - R) * tint_factor)
            Gi = int(G + (255 - G) * tint_factor)
            Bi = int(B + (255 - B) * tint_factor)
            color = rgb2hex(Ri, Gi, Bi)

            draw_params.time_begin = time_begin+i
            draw_params.time_end = time_begin+i+1
            draw_params.occupancy.shape.facecolor = color
            draw_params.occupancy.shape.edgecolor = color
            for shadow in shadows:
                shadow.draw(rnd, draw_params=draw_params)

    def plot(self,
             scenario=None,
             time_begin=0,
             time_end=500,
             ego_vehicle=None,
             sensor_view=None):

        draw_params = MPDrawParams().load(file_path="python/draw_params/scenario.yaml")
        draw_params.time_begin = time_begin
        draw_params.time_end = time_end

        # Set global draw params for drawing
        rnd = MPRenderer(figsize=(8, 8))
        rnd.draw_params = draw_params

        if sensor_view is not None:
            # Draw params can be overwritten when rendering specific objects
            draw_params = ShapeParams.load(
                file_path="python/draw_params/sensor_view.yaml", validate_types=False)
            ShapelyPolygon2Polygon(sensor_view).draw(
                rnd, draw_params=draw_params)

        if scenario is not None:
            if ego_vehicle is not None:
                scenario.remove_obstacle(ego_vehicle)

                draw_params = DynamicObstacleParams.load(
                    file_path="python/draw_params/ego_vehicle.yaml", validate_types=False)
                draw_params.time_begin = time_begin
                draw_params.time_end = time_end
                ego_vehicle.draw(rnd, draw_params=draw_params)

            shadow_obstacles = scenario.obstacles_by_role_and_type(
                obstacle_type=ObstacleType.UNKNOWN)
            scenario.remove_obstacle(shadow_obstacles)

            self.draw_shadows(rnd, shadow_obstacles, time_begin, 20)
            scenario.draw(rnd)

            scenario.add_objects(shadow_obstacles)
            if ego_vehicle is not None:
                scenario.add_objects(ego_vehicle)

        rnd.render()

    def plot_polyhedron_hull(self, polyhedron, color, figure):
        hull = ConvexHull(polyhedron)
        for s in hull.simplices:
            tri = Poly3DCollection([polyhedron[s]])
            tri.set_color(color)
            tri.set_alpha(0.5)
            figure.add_collection3d(tri)

    def plot_3D_shadows(self, shadow, sim_length):
        ID = shadow[0]
        polyhedra = shadow[1]

        figure = plt.figure()
        ax = figure.add_subplot(111, projection="3d")
        R = int(255)
        G = int(0)
        B = int(0)

        for polyhedron in polyhedra[::5]:
            timestep = polyhedron[0]
            poly = np.array(polyhedron[1])

            tint_factor = timestep/sim_length
            Ri = int(R - 255 * tint_factor)
            Bi = int(B + 255 * tint_factor)
            color = rgb2hex(Ri, G, Bi)

            hull = ConvexHull(poly)
            for s in hull.simplices:
                tri = Poly3DCollection([poly[s]])
                tri.set_color(color)
                tri.set_alpha(0.1)
                ax.add_collection3d(tri)
        plt.show()