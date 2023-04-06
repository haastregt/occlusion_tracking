import matplotlib.pyplot as plt

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
            file_path="draw_params/shadow.yaml", validate_types=False)
        draw_params.time_begin = time_begin
        draw_params.time_end = time_begin + 1
        for shadow in shadows:
            shadow.draw(rnd, draw_params=draw_params)

        # Draw the shadow predictions
        draw_params = DynamicObstacleParams.load(
            file_path="draw_params/shadow_prediction.yaml", validate_types=False)
        for i in reversed(range(time_horizon)):
            tint_factor = 0.005**(1/(i+1))
            Ri = int(R + (255 - R) * tint_factor)
            Gi = int(G + (255 - G) * tint_factor)
            Bi = int(B + (255 - G) * tint_factor)
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

        draw_params = MPDrawParams().load(file_path="draw_params/scenario.yaml")
        draw_params.time_begin = time_begin
        draw_params.time_end = time_end

        # Set global draw params for drawing
        rnd = MPRenderer(figsize=(8, 8))
        rnd.draw_params = draw_params

        if sensor_view is not None:
            # Draw params can be overwritten when rendering specific objects
            draw_params = ShapeParams.load(
                file_path="draw_params/sensor_view.yaml", validate_types=False)
            ShapelyPolygon2Polygon(sensor_view).draw(
                rnd, draw_params=draw_params)

        if scenario is not None:
            if ego_vehicle is not None:
                scenario.remove_obstacle(ego_vehicle)

                draw_params = DynamicObstacleParams.load(
                    file_path="draw_params/ego_vehicle.yaml", validate_types=False)
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
