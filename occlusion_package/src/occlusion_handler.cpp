#include "../include/cpp_occlusions/occlusion_handler.h"

#include "../include/cpp_occlusions/polyhedron_modifiers.h"

namespace cpp_occlusions
{

OcclusionHandler::OcclusionHandler(std::list<Polygon> driving_corridor_polygons, Polygon initial_sensor_view,
                                   int init_time_step, ReachabilityParams params)
{
    // Assert that Polygons are not self-intersecting: .is_simple()
    // Assert that Polygons have at least three points: .size()

    Polyhedron P;
    InitialiseAsExtrudedPolygon<HalfedgeDS> extrude(initial_sensor_view,
                                                    std::pair<float, float>{params.vmin, params.vmax});
    P.delegate(extrude);

    std::cout << "The polyhedron is " << (P.is_closed() ? "" : "not ") << "closed."
              << "\n";

    std::cout << "The polyhedron is " << (P.is_valid() ? "" : "not ") << "valid."
              << "\n";

    std::cout << "The polyhedron has " << P.size_of_vertices() << " vertices."
              << "\n";
    // Calculate the first view:
    // For each driving corridor:
    //     Take differences between driving corridor and sensorview
    //     Extrude differences to full range of longitudinal vel
    //     Append all separate polyhedrons to shadow list
    CGAL::draw(P);
}

OcclusionHandler::~OcclusionHandler()
{
}

void OcclusionHandler::Update(Polygon sensor_view)
{
    // This function updates the occlusion polyhedrons for the whole scene

    // t = new_time_step - self.time_step
    // self.time_step = new_time_step

    // Extrude sensor_view to full range of longitudinal vel
    //
    // For each shadow in list:
    //     shadow.expand(t)
    //     new_shadow_list.append = Boolean operation shadow and sensor_view (Note
    //     this could give multiple new polyhedra as they could have splitted)
    // shadow_list = new_shadow_list

    for (auto it = sensor_view.vertices_begin(); it != sensor_view.vertices_end(); ++it)
    {
        std::cout << *it << std::endl;
    }

    // check if the polygon is simple.
    std::cout << "The polygon is " << (sensor_view.is_simple() ? "" : "not ") << "simple."
              << "\n";
    // check if the polygon is convex
    std::cout << "The polygon is " << (sensor_view.is_convex() ? "" : "not ") << "convex."
              << "\n";

    CGAL::draw(sensor_view);
}

std::list<std::list<Polygon>> OcclusionHandler::GetReachableSets()
{
    // For each shadow in shadowlist:
    //     occupancy_set = shadow.get_cr_occupancy_set(time_step, dt,
    //     prediction_horizon) Use this occupancy set to set up dynamic obstacle

    // First element should be shadow shape (occupancy t=0), the rest the
    // occupancies of future time stepsd

    Polygon p1;
    p1.push_back(Point2(0, 0));
    p1.push_back(Point2(4, 0));
    p1.push_back(Point2(4, 4));
    p1.push_back(Point2(2, 2));
    p1.push_back(Point2(0, 4));

    Polygon p2;
    p2.push_back(Point2(4, 0));
    p2.push_back(Point2(4, 4));
    p2.push_back(Point2(2, 2));

    Polygon p3;
    p3.push_back(Point2(2, 2));
    p3.push_back(Point2(0, 0));
    p3.push_back(Point2(4, 0));

    std::list<Polygon> between;
    between.push_back(p1);
    between.push_back(p2);
    between.push_back(p3);
    std::list<std::list<Polygon>> temp;
    temp.push_back(between);
    temp.push_back(between);

    std::cout << "This works! \n";
    return temp;
}

} // namespace cpp_occlusions