#include "cpp_occlusions/occlusion_handler.h"
#include "cpp_occlusions/driving_corridor.h"
#include "cpp_occlusions/utility.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <chrono>

namespace cpp_occlusions
{

OcclusionHandler::OcclusionHandler(std::list<Polygon> driving_corridor_polygons,
                                   std::list<Polygon> mapped_driving_corridor_polygons, 
                                   std::list<std::list<Polygon>> lanes_in_driving_corridors,
                                   Polygon initial_sensor_view,
                                   int init_time_step, ReachabilityParams params)
    : _params(params), _time_step(init_time_step)
{
    _ID_allocator = 0;
    _time_step = 0;

    _update_time = 0;
    _prediction_time = 0;
    _num_updates = 0;
    _num_predictions = 0;

    Polyhedron P;
    std::list<CGAL::Polygon_with_holes_2<Kernel>> output_list;
    ExtrudeZ<HalfedgeDS> extrude(Polygon(), std::pair<float, float>{params.vmin, params.vmax});

    if (!initial_sensor_view.is_counterclockwise_oriented())
    {
        initial_sensor_view.reverse_orientation();
    }

    std::list<Polygon>::iterator mapped_it = mapped_driving_corridor_polygons.begin();
    std::list<std::list<Polygon>>::iterator lanes_it = lanes_in_driving_corridors.begin();
    for (Polygon driving_corridor_poly : driving_corridor_polygons)
    {
        if (!driving_corridor_poly.is_counterclockwise_oriented())
        {
            driving_corridor_poly.reverse_orientation();
            mapped_it->reverse_orientation();
        }
        assert(mapped_it->is_counterclockwise_oriented() &&
               "original and mapped driving corridors should have same orientation");

        // TODO: This technically creates a memory leak. Maybe add these pointers to an array
        // for which each element will be deleted in the OcclusionHandler destructor. However
        // since there is just one Occlusion Handler per simulation, it should not really matter.
        DrivingCorridor *driving_corridor =
            new DrivingCorridor(driving_corridor_poly, *mapped_it, *lanes_it, 1 / _params.mapping_quality);

        std::list<OccludedVolume> corridor;

        output_list.clear();
        CGAL::difference(driving_corridor_poly, initial_sensor_view, std::back_inserter(output_list));

        for (CGAL::Polygon_with_holes_2<Kernel> diff : output_list)
        {
            assert(diff.outer_boundary().is_simple() && "Polygon has a self-intersection!");
            assert((diff.outer_boundary().size() > 2) && "Polygon should have at least three points");

            extrude.polygon = diff.outer_boundary();
            P = Polyhedron();
            P.delegate(extrude);

            assert(P.is_closed() && "Polyhedra should be closed in order for conversion to Nef");

            ++_ID_allocator;
            corridor.push_back(OccludedVolume(P, driving_corridor, _params, _ID_allocator));
            if (_params.export_shadows)
            {
                SaveShadow(_ID_allocator, P);
            }
        }

        if (!corridor.empty())
        {
            _shadow_list_by_corridor.push_back(corridor);
        }

        ++mapped_it;
    }
}

OcclusionHandler::~OcclusionHandler()
{
}

void OcclusionHandler::Update(Polygon sensor_view, int new_time_step)
{
    _num_updates++;
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();

    float dt = _params.dt * (new_time_step - _time_step);
    _time_step = new_time_step;

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
        DrivingCorridor *driving_corridor = corridor.begin()->GetDrivingCorridor();

        std::list<Nef_polyhedron> nef_list;
        std::list<int> ID_list;
        for (OccludedVolume shadow : corridor)
        {
            for (OccludedVolume new_shadow : shadow.Propagate(dt, sensor_view))
            {
                // Check if this intersects with another shadow in this corridor, if so merge
                bool is_double = false;
                Nef_polyhedron nef_new(new_shadow.GetPolyhedron());

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
                    if(std::find(ID_list.begin(), ID_list.end(), new_shadow._ID) != ID_list.end())
                    {
                        ID_list.push_back(new_shadow._ID);
                    }
                    else
                    {
                        ++_ID_allocator;
                        ID_list.push_back(new_shadow._ID);
                    }
                }
            }
        }

        std::list<OccludedVolume> new_corridor;

        std::list<int>::iterator ID_it = ID_list.begin();
        for (Nef_polyhedron nef : nef_list)
        {
            Polyhedron P;
            nef.convert_to_polyhedron(P);
            new_corridor.push_back(OccludedVolume(P, driving_corridor, _params, *ID_it));
            if (_params.export_shadows)
            {
                SaveShadow(*ID_it, P);
            }
            ++ID_it;
        }

        _shadow_list_by_corridor.push_back(new_corridor);
    }

    std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
    _update_time += std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
}

std::list<std::list<Polygon>> OcclusionHandler::GetReachableSets()
{
    _num_predictions++;
    std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();

    std::list<std::list<Polygon>> occupancy_lists;
    for (auto shadow_list : _shadow_list_by_corridor)
    {
        for (OccludedVolume shadow : shadow_list)
        {
            occupancy_lists.push_back(shadow.ComputeFutureOccupancies());
        }
    }
    
    std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
    _prediction_time += std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
    
    return occupancy_lists;
}

void OcclusionHandler::SaveShadow(int ID, Polyhedron polyhedron)
{
    bool exists = false;

    for (auto it = _shadow_saves.begin(); it != _shadow_saves.end(); ++it) {
        if (std::get<0>(*it) == ID) {
            std::get<1>(*it).push_back(std::tuple<int, Polyhedron>{_time_step, polyhedron}); 
            exists = true;
            break;
        }
    }
    if (!exists)
    {
        std::tuple<int, std::list<std::tuple<int,Polyhedron>>> shadow;
        std::get<0>(shadow) = ID;
        std::get<1>(shadow).push_back(std::tuple<int, Polyhedron>{_time_step, polyhedron});
        _shadow_saves.push_back(shadow);
    }
}

std::list<std::tuple<int, std::list<std::tuple<int, std::list<std::list<float>>>>>> OcclusionHandler::ExportShadows()
{
    std::list<std::tuple<int, std::list<std::tuple<int, std::list<std::list<float>>>>>> export_results;
    for(auto item : _shadow_saves)
    {
        std::tuple<int, std::list<std::tuple<int, std::list<std::list<float>>>>> shadow_list;
        std::get<0>(shadow_list) = std::get<0>(item);

        std::list<std::tuple<int, std::list<std::list<float>>>> shadows;
        for (auto poly : std::get<1>(item))
        {
            std::tuple<int, std::list<std::list<float>>> export_poly;
            std::get<0>(export_poly) = std::get<0>(poly);

            std::list<std::list<float>> export_point_list;
            for(auto it = std::get<1>(poly).points_begin(); it != std::get<1>(poly).points_end(); ++it)
            {
                std::list<float> export_point;
                
                export_point.push_back(CGAL::to_double(it->x()));
                export_point.push_back(CGAL::to_double(it->y()));
                export_point.push_back(CGAL::to_double(it->z()));

                export_point_list.push_back(export_point);
            }

            std::get<1>(export_poly) = export_point_list;

            shadows.push_back(export_poly);
        }
        std::get<1>(shadow_list) = shadows;

        export_results.push_back(shadow_list);
    }

    return export_results;
}

std::tuple<double, double> OcclusionHandler::ExportComputationalTime()
{
    return std::tuple<double,double>{_update_time/_num_updates, _prediction_time/_num_predictions};
}


} // namespace cpp_occlusions