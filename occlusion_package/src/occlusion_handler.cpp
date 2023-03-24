#include <CGAL/Boolean_set_operations_2.h>

#include <cassert>

#include "../include/cpp_occlusions/occlusion_handler.h"

#include "../include/cpp_occlusions/polyhedron_modifiers.h"

namespace cpp_occlusions
{

OcclusionHandler::OcclusionHandler(std::list<Polygon> driving_corridor_polygons, Polygon initial_sensor_view,
                                   int init_time_step, ReachabilityParams params)
{
    Polygon temp;
    Polygon *polygon_ptr = &temp;
    Polyhedron P;
    std::list<CGAL::Polygon_with_holes_2<Kernel>> output_list;
    InitialiseAsExtrudedPolygon<HalfedgeDS> extrude(Polygon(), std::pair<float, float>{params.vmin, params.vmax});

    if (!initial_sensor_view.is_counterclockwise_oriented())
    {
        initial_sensor_view.reverse_orientation();
    }

    for (Polygon driving_corridor : driving_corridor_polygons)
    {
        if (!driving_corridor.is_counterclockwise_oriented())
        {
            driving_corridor.reverse_orientation();
        }

        _driving_corridors.push_back(driving_corridor);
        output_list.clear(); // Empty the output_list

        CGAL::difference(driving_corridor, initial_sensor_view, std::back_inserter(output_list));
        for (CGAL::Polygon_with_holes_2<Kernel> diff : output_list)
        {
            assert(diff.outer_boundary().is_simple() && "Polygon has a self-intersection!");
            assert((diff.outer_boundary().size() > 2) && "Polygon should have at least three points");

            P = Polyhedron(); // Re-initialising an empty polyhedron
            extrude.polygon_ref = diff.outer_boundary();

            for (auto it = diff.outer_boundary().vertices_begin(); it != diff.outer_boundary().vertices_end(); ++it)
            {
                std::cout << "This works for diff.outer_boundary(): " << *it << std::endl;
            }

            for (auto it = extrude.polygon_ref.vertices_begin(); it != extrude.polygon_ref.vertices_end(); ++it)
            {
                std::cout << "This works for extrude.polygon_ref: " << *it << std::endl;
            }

            P.delegate(extrude);
            std::cout << "Delegate succeeded" << std::endl;
            _shadow_list.push_back(OccludedVolume(P, *_driving_corridors.end()));
        }
    }

    for (OccludedVolume shadow : _shadow_list)
    {
        CGAL::draw(shadow._shadow_polyhedron);
    }
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