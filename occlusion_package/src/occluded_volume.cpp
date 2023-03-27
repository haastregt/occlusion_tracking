//#include <CGAL/Polygon_mesh_processing/corefinement.h>
//#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
//#include <CGAL/Surface_mesh.h>

#include <cassert>

#include "../include/cpp_occlusions/occluded_volume.h"

#include "../include/cpp_occlusions/polyhedron_modifiers.h"

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/draw_polyhedron.h>
#include <CGAL/minkowski_sum_3.h>

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
    int n = 16;                                  // Number of vertices to approximate the circle
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
    CGAL::draw(disk);

    Nef_polyhedron nef_disk(disk);
    Nef_polyhedron nef(polyhedron);
    return CGAL::minkowski_sum_3(nef, nef_disk);
}

Nef_polyhedron OccludedVolume::VelocityAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron)
{
    Polyhedron placeholder;
    return placeholder;
}

Nef_polyhedron OccludedVolume::AccelerationAbstraction(float dt, Polyhedron polyhedron)
{
    Polyhedron placeholder;
    return placeholder;
}

Nef_polyhedron OccludedVolume::AccelerationAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron)
{
    Polyhedron placeholder;
    return placeholder;
}

std::list<OccludedVolume> OccludedVolume::Propagate(float dt, Polygon &sensor_view)
{
    std::list<OccludedVolume> new_shadow_list;

    Nef_polyhedron new_shadow(Nef_polyhedron::COMPLETE);
    Polyhedron P;

    // TODO: if (_params.use_abstraction.velocity)
    Nef_polyhedron velocity_abstraction = VelocityAbstraction(dt, _shadow_polyhedron);
    new_shadow *= velocity_abstraction;

    // TODO: if (_params.use_abstraction.acceleration)
    // Nef_polyhedron acceleration_abstraction = AccelerationAbstraction(dt, _shadow_polyhedron);
    // new_shadow *= acceleration_abstraction;

    // TODO: Include other abstractions here in a similar matter

    Nef_polyhedron copy;
    std::list<CGAL::Polygon_with_holes_2<Kernel>> output_list;
    InitialiseAsExtrudedPolygon<HalfedgeDS> extrude(Polygon(), std::pair<float, float>{_params.vmin, _params.vmax});
    CGAL::difference(_road_polygon, sensor_view, std::back_inserter(output_list));
    for (CGAL::Polygon_with_holes_2<Kernel> diff : output_list)
    {
        assert(diff.outer_boundary().is_simple() && "Polygon has a self-intersection!");
        assert((diff.outer_boundary().size() > 2) && "Polygon should have at least three points");

        copy = new_shadow;
        P = Polyhedron();

        extrude.polygon = diff.outer_boundary();
        P.delegate(extrude);
        assert(P.is_closed() && "Polyhedra should be closed in order for conversion to Nef");
        copy *= Nef_polyhedron(P);

        assert(copy.is_simple() && "Nef_Polyhedra should be simple in order for conversion to normal polyhedra");

        P = Polyhedron();
        copy.convert_to_polyhedron(P);
        // TODO: If volume > min_volume
        new_shadow_list.push_back(OccludedVolume(P, _road_polygon, _params));
        CGAL::draw(P);
    }

    return new_shadow_list;
}

std::list<Polygon> OccludedVolume::ComputeFutureOccupancies(float dt, int prediction_horizon)
{
    // This function performs the reachability of the shadow over a prediction
    // horizon and returns the occupancy set for a dynamic obstacle in CommonRoad

    // polyhedron = self.polyhedron # Make a copy
    // occupancy_set = []
    // for i in range(prediction_horizon):
    //   expand(dt, polyhedron)
    //   projected = xy_project(polyhedron)
    //   occupancy_set.append(projected)

    std::list<Polygon> placeholder;
    return placeholder;
}

} // namespace cpp_occlusions