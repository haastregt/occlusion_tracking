#include "cpp_occlusions/occluded_volume.h"
#include "cpp_occlusions/utility.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/minkowski_sum_3.h>

#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polyhedron.h>

namespace cpp_occlusions
{

OccludedVolume::OccludedVolume(Polyhedron initial_polyhedron, const Polygon road_polygon,
                               const ReachabilityParams params)
    : _shadow_polyhedron(initial_polyhedron), _road_polygon(road_polygon), _params(params)
{
}

OccludedVolume::~OccludedVolume()
{
}

Nef_polyhedron OccludedVolume::VelocityAbstraction(float dt, Polyhedron polyhedron)
{
    int n = 8;                                   // Number of vertices to approximate the circle
    float R = _params.vmax * dt / cos(M_PI / n); // Use apothem to make sure we have an over-approximation of the circle
    Polygon disk_polygon;
    for (int i = 0; i < n; ++i)
    {
        float angle = -i * 2 * M_PI / n;
        float x = R * cos(angle);
        float y = R * sin(angle);
        disk_polygon.push_back(Point2(x, y));
    }
    Polyhedron disk;

    InitialiseAsExtrudedPolygon<HalfedgeDS> make_polyhedron(disk_polygon, std::pair<float, float>{0, 1E-15});
    disk.delegate(make_polyhedron);

    Nef_polyhedron nef_disk(disk);
    Nef_polyhedron nef(polyhedron);
    return CGAL::minkowski_sum_3(nef, nef_disk);
}

Nef_polyhedron OccludedVolume::VelocityAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron)
{
    Nef_polyhedron placeholder;
    return placeholder;
}

Nef_polyhedron OccludedVolume::AccelerationAbstraction(float dt, Polyhedron polyhedron)
{
    Nef_polyhedron placeholder;
    return placeholder;
}

Nef_polyhedron OccludedVolume::AccelerationAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron)
{
    Nef_polyhedron placeholder;
    return placeholder;
}

std::list<OccludedVolume> OccludedVolume::Propagate(float dt, Polygon &sensor_view)
{
    std::list<OccludedVolume> new_shadow_list;

    Nef_polyhedron new_shadow(Nef_polyhedron::COMPLETE);
    Polyhedron P;
    CGAL::Surface_mesh<Nef_polyhedron::Point_3> surface_mesh;

    if (_params.velocity_tracking_enabled)
    {
        // TODO:
        // Nef_polyhedron acceleration_abstraction = AccelerationAbstraction(dt, _shadow_polyhedron);
        // new_shadow *= acceleration_abstraction;
    }
    else
    {
        Nef_polyhedron velocity_abstraction = VelocityAbstraction(dt, _shadow_polyhedron);
        new_shadow *= velocity_abstraction;
    }

    Nef_polyhedron copy;
    std::list<CGAL::Polygon_with_holes_2<Kernel>> output_list;
    InitialiseAsExtrudedPolygon<HalfedgeDS> extrude(Polygon(), std::pair<float, float>{_params.vmin, _params.vmax});
    CGAL::difference(_road_polygon, sensor_view, std::back_inserter(output_list));

    for (CGAL::Polygon_with_holes_2<Kernel> diff : output_list)
    {
        assert(diff.outer_boundary().is_simple() && "Polygon has a self-intersection!");
        assert((diff.outer_boundary().size() > 2) && "Polygon should have at least three points");

        copy = Nef_polyhedron(new_shadow);
        P = Polyhedron();

        extrude.polygon = diff.outer_boundary();
        P.delegate(extrude);
        assert(P.is_closed() && "Polyhedra should be closed in order for conversion to Nef");
        copy *= Nef_polyhedron(P);

        assert(copy.is_simple() && "Nef_Polyhedra should be simple in order for conversion to normal polyhedra");

        // We check if the minimal volume is above a treshhold to avoid small shadows due to numerical rounding
        // TODO: Maybe not necessary since we use exact construction kernel
        surface_mesh = CGAL::Surface_mesh<Nef_polyhedron::Point_3>();
        CGAL::convert_nef_polyhedron_to_polygon_mesh(copy, surface_mesh, true);

        if (CGAL::Polygon_mesh_processing::volume(surface_mesh) > _params.min_shadow_volume)
        {
            P = Polyhedron();
            copy.convert_to_polyhedron(P);

            DissolveCloseVertices(P, 0.001);
            new_shadow_list.push_back(OccludedVolume(P, _road_polygon, _params));
        }
    }

    return new_shadow_list;
}

std::list<Polygon> OccludedVolume::ComputeFutureOccupancies()
{
    std::list<Polygon> occupancy_set;
    occupancy_set.push_back(ProjectXY(_shadow_polyhedron));

    Polyhedron P;
    InitialiseAsExtrudedPolygon<HalfedgeDS> extrude(_road_polygon, std::pair<float, float>{_params.vmin, _params.vmax});
    P.delegate(extrude);
    Nef_polyhedron nef_road(P);

    int num_predictions = _params.prediction_horizon / _params.prediction_interval;
    for (int i = 1; i <= num_predictions; i++)
    {
        Nef_polyhedron occupancy(Nef_polyhedron::COMPLETE);

        if (_params.velocity_tracking_enabled)
        {
            // TODO: Other abstractions
        }
        else
        {
            Nef_polyhedron velocity_abstraction =
                VelocityAbstraction(_params.dt * _params.prediction_interval * i, _shadow_polyhedron);
            occupancy *= velocity_abstraction;
        }

        occupancy *= nef_road;

        P = Polyhedron();
        occupancy.convert_to_Polyhedron(P);
        occupancy_set.push_back(ProjectXY(P));
    }

    std::list<Polygon> placeholder;
    return occupancy_set;
}

} // namespace cpp_occlusions