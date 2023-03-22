#include "../include/cpp_occlusions/occluded_volume.h"

#include "../include/cpp_occlusions/polyhedron_modifiers.h"

namespace cpp_occlusions
{

OccludedVolume::OccludedVolume(Polyhedron initial_polyhedron, Polygon road_polygon)
{
    _shadow_polyhedron = initial_polyhedron;
    //_road_polyhedron = extrude(road_polygon);
}

OccludedVolume::~OccludedVolume()
{
}

void OccludedVolume::Propagate(float dt)
{
    // This function expands the shadow polyhedron using reachability

    // if not polyhedron: polyhedron = self.polyhedron

    // Note that this proposed expansion algorithm is an over - approximation.The
    // proper reachable set computations are more advanced(maybe see later if it
    // is possible)

    // Abstractions as various functions and take union here?

    // a : skew along x with each vertex's Z-coord (which is vx)*dt +
    // 1/2*a_max*dt^2
    // b : skew along x with each vertex's Z-coord (which is vx)*dt
    // - 1/2*a_min*dt^2
    // Take union of a &b Extrude in Z - dir by a_max *dt on top and a_min *dt on
    // bottom
    // Extrude in Y - dit by y_vel *dt in both directions

    // Take intersection between expanded poly and extruded driving corridor

    // Maybe the extrusions can be done by taking a minkowsky sum with a vector
}

void OccludedVolume::Propagate(float dt, Polyhedron &polyhedron)
{
}

std::vector<Polygon> OccludedVolume::ComputeFutureOccupancies(float dt, int prediction_horizon)
{
    // This function performs the reachability of the shadow over a prediction
    // horizon and returns the occupancy set for a dynamic obstacle in CommonRoad

    // polyhedron = self.polyhedron # Make a copy
    // occupancy_set = []
    // for i in range(prediction_horizon):
    //   expand(dt, polyhedron)
    //   projected = xy_project(polyhedron)
    //   occupancy_set.append(projected)

    std::vector<Polygon> placeholder(prediction_horizon);
    return placeholder;
}

} // namespace cpp_occlusions