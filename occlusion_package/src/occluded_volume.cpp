#include "cpp_occlusions/occluded_volume.h"
#include "cpp_occlusions/utility.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/minkowski_sum_3.h>

#include <CGAL/draw_nef_3.h>
#include <CGAL/draw_polyhedron.h>

namespace cpp_occlusions
{

OccludedVolume::OccludedVolume(Polyhedron initial_polyhedron, DrivingCorridor *driving_corridor,
                               const ReachabilityParams params)
    : _shadow_polyhedron(initial_polyhedron), _driving_corridor(driving_corridor), _params(params)
{
}

OccludedVolume::~OccludedVolume()
{
}

Nef_polyhedron OccludedVolume::VelocityAbstraction(float dt, Polyhedron polyhedron)
{
    int n = 4;                                   // Number of vertices to approximate the halfcircle
    float R = _params.vmax * dt / cos(M_PI / n); // Use apothem to make sure we have an over-approximation of the circle
    Polygon disk_polygon;
    for (int i = 0; i < n; ++i)
    {
        float angle = i * M_PI / n - 0.5 * M_PI;
        float x = R * cos(angle);
        float y = R * sin(angle);
        disk_polygon.push_back(Point2(x, y));
    }
    Polyhedron disk;

    InitialiseAsExtrudedPolygon<HalfedgeDS> make_polyhedron(disk_polygon, std::pair<float, float>{0, 1E-15});
    disk.delegate(make_polyhedron);

    // TODO: Some double conversions happening? Probably the transformations can use
    // Nef as inputs and outputs
    CGAL::draw(polyhedron);
    Polyhedron mapped_polyhedron = _driving_corridor->TransformOriginalToMapped(polyhedron);

    std::cout << "Is mapped polyhedron valid? " << mapped_polyhedron.is_valid() << std::endl;
    std::cout << "Is mapped polyhedron closed? " << mapped_polyhedron.is_closed() << std::endl;
    std::cout << "Is mapped polyhedron empty? " << mapped_polyhedron.is_empty() << std::endl;
    std::cout << "Number of vertices of mapped polyhedron: " << mapped_polyhedron.size_of_vertices() << std::endl;

    Nef_polyhedron nef_disk(disk);
    Nef_polyhedron nef(mapped_polyhedron);

    std::cout << "Is nef valid? " << nef.is_valid() << std::endl;
    std::cout << "Is nef simple? " << nef.is_simple() << std::endl;
    std::cout << "Is nef empty? " << nef.is_empty() << std::endl;
    std::cout << "Is nef space?" << nef.is_space() << std::endl;
    std::cout << "Number of vertices " << nef.number_of_vertices() << std::endl;
    std::cout << "Number of volumes " << nef.number_of_volumes() << std::endl;

    Polyhedron convex_hull;
    CGAL::convex_hull_3(mapped_polyhedron.points_begin(), mapped_polyhedron.points_end(), convex_hull);

    nef = Nef_polyhedron(convex_hull);
    CGAL::draw(nef);

    Nef_polyhedron minkowski_sum = CGAL::minkowski_sum_3(nef, nef_disk);

    Polyhedron mapped_result;
    minkowski_sum.convert_to_polyhedron(mapped_result);
    CGAL::draw(mapped_result);

    Polyhedron result = _driving_corridor->TransformMappedToOriginal(mapped_result);
    CGAL::draw(result);

    Polyhedron convex_result;
    CGAL::convex_hull_3(result.points_begin(), result.points_end(), convex_result);
    CGAL::draw(convex_result);
    return Nef_polyhedron(convex_result);
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
    CGAL::difference(_driving_corridor->original_polygon, sensor_view, std::back_inserter(output_list));

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
            new_shadow_list.push_back(OccludedVolume(P, _driving_corridor, _params));
        }
    }

    return new_shadow_list;
}

std::list<Polygon> OccludedVolume::ComputeFutureOccupancies()
{
    std::list<Polygon> occupancy_set;
    std::cout << "Projecting original polyhedron" << std::endl;
    occupancy_set.push_back(ProjectXY(_shadow_polyhedron));

    Polyhedron P;
    std::cout << "Setting up extrusion. dc adress: " << _driving_corridor << std::endl;
    InitialiseAsExtrudedPolygon<HalfedgeDS> extrude(_driving_corridor->original_polygon,
                                                    std::pair<float, float>{_params.vmin, _params.vmax});
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
            std::cout << "Going to compute velocity abstraction" << std::endl;
            Nef_polyhedron velocity_abstraction =
                VelocityAbstraction(_params.dt * _params.prediction_interval * i, _shadow_polyhedron);
            occupancy *= velocity_abstraction;
        }
        // TODO: This is redundant since the mapping used in the Velocity abstraction already
        // takes the intersection with the road
        occupancy *= nef_road;

        P = Polyhedron();
        occupancy.convert_to_Polyhedron(P);
        std::cout << "Going to Project polyhedron " << std::endl;
        std::cout << "Is the polyhedron valid? " << P.is_valid() << std::endl;
        std::cout << "Is the polyhedron closed? " << P.is_closed() << std::endl;
        std::cout << "Is the polyhedron empty? " << P.is_empty() << std::endl;

        if (P.is_empty())
        {
            Debug(i);
        }

        occupancy_set.push_back(ProjectXY(P));
        std::cout << "Projection completed" << std::endl;
    }

    return occupancy_set;
}

void OccludedVolume::Debug(int i)
{
    Nef_polyhedron occupancy(Nef_polyhedron::COMPLETE);

    int n = 4; // Number of vertices to approximate the halfcircle
    float R = _params.vmax * _params.dt /
              cos(M_PI / n); // Use apothem to make sure we have an over-approximation of the circle
    Polygon disk_polygon;
    for (int i = 0; i < n; ++i)
    {
        float angle = i * M_PI / n - 0.5 * M_PI;
        float x = R * cos(angle);
        float y = R * sin(angle);
        disk_polygon.push_back(Point2(x, y));
    }
    Polyhedron disk;

    InitialiseAsExtrudedPolygon<HalfedgeDS> make_polyhedron(disk_polygon, std::pair<float, float>{0, 1E-15});
    disk.delegate(make_polyhedron);

    CGAL::draw(disk);

    // TODO: Some double conversions happening? Probably the transformations can use
    // Nef as inputs and outputs
    Polyhedron mapped_polyhedron = _driving_corridor->TransformOriginalToMapped(_shadow_polyhedron);

    CGAL::draw(_shadow_polyhedron);
    CGAL::draw(mapped_polyhedron);

    Nef_polyhedron nef_disk(disk);
    Nef_polyhedron nef(mapped_polyhedron);
    Nef_polyhedron minkowski_sum = CGAL::minkowski_sum_3(nef, nef_disk);

    Polyhedron mapped_result;
    minkowski_sum.convert_to_polyhedron(mapped_result);

    Polyhedron result = _driving_corridor->TransformMappedToOriginal(mapped_result);
    CGAL::draw(mapped_result);
    CGAL::draw(result);
}

Polyhedron OccludedVolume::GetPolyhedron()
{
    return _shadow_polyhedron;
}

DrivingCorridor *OccludedVolume::GetDrivingCorridor()
{
    return _driving_corridor;
}

} // namespace cpp_occlusions