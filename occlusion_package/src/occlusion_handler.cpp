#include <CGAL/Boolean_set_operations_2.h>

#include <cassert>

#include "../include/cpp_occlusions/occlusion_handler.h"

#include "../include/cpp_occlusions/poly_modifiers.h"

#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polyhedron.h>
namespace cpp_occlusions
{

OcclusionHandler::OcclusionHandler(std::list<Polygon> driving_corridor_polygons, Polygon initial_sensor_view,
                                   int init_time_step, ReachabilityParams params)
    : _params(params), _time_step(init_time_step)
{
    int num_shadows = 0;
    _time_step = 0;

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

        std::list<OccludedVolume> corridor;

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

            corridor.push_back(OccludedVolume(P, driving_corridor, _params));
        }

        if (!corridor.empty())
        {
            num_shadows += corridor.size();
            _shadow_list_by_corridor.push_back(corridor);
        }
    }

    // for debugging exploding shadows
    // std::cout << "Initialised with " << num_shadows << " initial shadows" << std::endl;
}

OcclusionHandler::~OcclusionHandler()
{
}

void OcclusionHandler::Update(Polygon sensor_view, int new_time_step)
{
    float dt = _params.dt * (new_time_step - _time_step);
    _time_step = new_time_step;

    int num_shadows = 0;

    // Check this since first time step of simulation is 0, while time at initialisation is also at 0.
    if (dt == 0)
        return;

    if (!sensor_view.is_counterclockwise_oriented())
    {
        sensor_view.reverse_orientation();
    }

    std::list<std::list<OccludedVolume>> copy_shadow_list_by_corridor = _shadow_list_by_corridor;
    _shadow_list_by_corridor.clear();
    for (auto corridor : copy_shadow_list_by_corridor)
    {
        Polygon road_polygon = corridor.begin()->_road_polygon;

        std::list<Nef_polyhedron> nef_list;
        for (OccludedVolume shadow : corridor)
        {
            for (OccludedVolume new_shadow : shadow.Propagate(dt, sensor_view))
            {
                // Check if this intersects with another shadow in this corridor, if so merge
                bool is_double = false;
                Nef_polyhedron nef_new(new_shadow._shadow_polyhedron);

                for (Nef_polyhedron &nef_existing : nef_list)
                {
                    if (nef_new * nef_existing != Nef_polyhedron::EMPTY)
                    {
                        nef_existing += nef_new;
                        is_double = true;
                        break;
                    }
                }
                if (!is_double)
                {
                    nef_list.push_back(nef_new);
                }
            }
        }

        std::list<OccludedVolume> new_corridor;

        for (Nef_polyhedron nef : nef_list)
        {
            Polyhedron P;
            nef.convert_to_polyhedron(P);
            new_corridor.push_back(OccludedVolume(P, road_polygon, _params));
        }
        num_shadows += new_corridor.size();

        _shadow_list_by_corridor.push_back(new_corridor);
    }

    // for debuggind exploding shadows
    // std::cout << "At time step " << new_time_step << "s we have " << num_shadows << " shadows" << std::endl;
}

std::list<std::list<Polygon>> OcclusionHandler::GetReachableSets()
{
    std::list<std::list<Polygon>> occupancy_lists;

    for (auto shadow_list : _shadow_list_by_corridor)
    {
        for (OccludedVolume shadow : shadow_list)
        {
            occupancy_lists.push_back(shadow.ComputeFutureOccupancies());
        }
    }

    return occupancy_lists;
}

} // namespace cpp_occlusions