#include "cpp_occlusions/occluded_volume.h"
#include "cpp_occlusions/utility.h"

#include <cmath>

#include <CGAL/Aff_transformation_3.h>
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
    Polyhedron mapped_polyhedron = _driving_corridor->TransformOriginalToMapped(polyhedron);
    Polyhedron convex_mapped;
    CGAL::convex_hull_3(mapped_polyhedron.points_begin(), mapped_polyhedron.points_end(), convex_mapped);

    Nef_polyhedron nef_disk(disk);
    Nef_polyhedron nef(convex_mapped);

    Nef_polyhedron minkowski_sum = CGAL::minkowski_sum_3(nef, nef_disk);

    Polyhedron mapped_result;
    minkowski_sum.convert_to_polyhedron(mapped_result);
    Polyhedron result = _driving_corridor->TransformMappedToOriginal(mapped_result);

    Polyhedron convex_result;
    CGAL::convex_hull_3(result.points_begin(), result.points_end(), convex_result);
    return Nef_polyhedron(convex_result);
}

Nef_polyhedron OccludedVolume::VelocityAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron)
{
    Nef_polyhedron placeholder;
    return placeholder;
}

Nef_polyhedron OccludedVolume::AccelerationAbstraction(float dt, Polyhedron polyhedron)
{
    // Acceleration abstraction: P = A*P + B*Ubounds, with A = [1,0,dt; 0,1,0; 0,0,1] and B = [0.5dt^2, 0; 0, dt; dt, 0]
    CGAL::Aff_transformation_3<Kernel> A(Kernel::RT(1), Kernel::RT(0), Kernel::RT(dt), Kernel::RT(0), Kernel::RT(1),
                                         Kernel::RT(0), Kernel::RT(0), Kernel::RT(0), Kernel::RT(1));

    // B*Ubounds is a rectangular cuboid:
    float xu = 0.5 * pow(dt, 2) * _params.amax;
    float xl = 0.5 * pow(dt, 2) * _params.amin;
    float yu = dt * _params.vlatmax;
    float yl = -dt * _params.vlatmax;
    float vu = dt * _params.amax;
    float vl = dt * _params.amin;

    Polygon rect;
    rect.push_back(Point2(xu, yu));
    rect.push_back(Point2(xl, yu));
    rect.push_back(Point2(xl, yl));
    rect.push_back(Point2(xu, yl));

    Polyhedron input;
    InitialiseAsExtrudedPolygon<HalfedgeDS> make_cuboid(rect, std::pair<float, float>{vl, vu});
    input.delegate(make_cuboid);

    // Also make a cuboid for the velocity bounds (This should maybe be its own abstraction for clarity)
    Polygon full;
    full.push_back(Point2(1E15, 1E15));
    full.push_back(Point2(-1E15, 1E15));
    full.push_back(Point2(-1E15, -1E15));
    full.push_back(Point2(1E15, -1E15));

    Polyhedron vbounds;
    make_cuboid.polygon = full;
    make_cuboid.bounds = std::pair<float, float>{_params.vmin, _params.vmax};
    vbounds.delegate(make_cuboid);

    // Now compute the abstraction
    Polyhedron P = _driving_corridor->TransformOriginalToMapped(polyhedron);
    CGAL::convex_hull_3(P.points_begin(), P.points_end(), P);

    std::transform(P.points_begin(), P.points_end(), P.points_begin(), A);

    Nef_polyhedron P_nef(P);
    Nef_polyhedron input_nef(input);
    P_nef = CGAL::minkowski_sum_3(P_nef, input_nef);

    Nef_polyhedron vbounds_nef(vbounds);
    P_nef *= vbounds_nef;

    P_nef.convert_to_polyhedron(P);
    P = _driving_corridor->TransformMappedToOriginal(P);
    CGAL::convex_hull_3(P.points_begin(), P.points_end(), P);

    return Nef_polyhedron(P);
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
        Nef_polyhedron acceleration_abstraction = AccelerationAbstraction(dt, _shadow_polyhedron);
        new_shadow *= acceleration_abstraction;
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
    occupancy_set.push_back(ProjectXY(_shadow_polyhedron));

    Polyhedron P;
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
            Nef_polyhedron velocity_abstraction =
                AccelerationAbstraction(_params.dt * _params.prediction_interval * i, _shadow_polyhedron);
            occupancy *= velocity_abstraction;
        }
        else
        {
            Nef_polyhedron velocity_abstraction =
                VelocityAbstraction(_params.dt * _params.prediction_interval * i, _shadow_polyhedron);
            occupancy *= velocity_abstraction;
        }
        // TODO: This is redundant since the mapping used in the Velocity abstraction already
        // takes the intersection with the road
        occupancy *= nef_road;

        P = Polyhedron();
        occupancy.convert_to_Polyhedron(P);

        occupancy_set.push_back(ProjectXY(P));
    }

    return occupancy_set;
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