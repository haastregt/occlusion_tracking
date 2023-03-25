#include <CGAL/Boolean_set_operations_2.h>

#include <cassert>

#include "../include/cpp_occlusions/occlusion_handler.h"

#include "../include/cpp_occlusions/polyhedron_modifiers.h"

namespace cpp_occlusions
{

OcclusionHandler::OcclusionHandler(std::list<Polygon> driving_corridor_polygons, Polygon initial_sensor_view,
                                   int init_time_step, ReachabilityParams params)
    : _params(params), _time_step(init_time_step)
{
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

        output_list.clear();
        CGAL::difference(driving_corridor, initial_sensor_view, std::back_inserter(output_list));

        for (CGAL::Polygon_with_holes_2<Kernel> diff : output_list)
        {
            assert(diff.outer_boundary().is_simple() && "Polygon has a self-intersection!");
            assert((diff.outer_boundary().size() > 2) && "Polygon should have at least three points");

            extrude.polygon = diff.outer_boundary();
            P = Polyhedron();
            P.delegate(extrude);

            assert(P.is_closed() && "Polyhedra should be closed in order for conversion to Nef");

            _shadow_list.push_back(OccludedVolume(P, *_driving_corridors.end(), _params));
        }
    }
}

OcclusionHandler::~OcclusionHandler()
{
}

void OcclusionHandler::Update(Polygon sensor_view, int new_time_step)
{
    float dt = new_time_step - _time_step;
    _time_step += dt;

    std::list<OccludedVolume> new_shadow_list;
    for (OccludedVolume shadow : _shadow_list)
    {
        for (OccludedVolume new_shadow : shadow.Propagate(dt, sensor_view))
        {
            new_shadow_list.push_back(new_shadow);
        }
    }
}

std::list<std::list<Polygon>> OcclusionHandler::GetReachableSets()
{
    std::list<std::list<Polygon>> occupancy_lists;

    for (OccludedVolume shadow : _shadow_list)
    {
        occupancy_lists.push_back(shadow.ComputeFutureOccupancies(_params.prediction_dt, _params.prediction_horizon));
    }

    return occupancy_lists;
}

} // namespace cpp_occlusions