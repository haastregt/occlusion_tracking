#include "type_definitions.h"

#ifndef REACHABILITY_CORE_H
#define REACHABILITY_CORE_H

namespace cpp_occlusions
{

class OccludedVolume
{
  private:
    Polyhedron _shadow_polyhedron;
    Polyhedron _road_polyhedron;

  public:
    OccludedVolume(Polyhedron initial_polyhedron, Polygon road_polygon);

    ~OccludedVolume();

    /// Propagate the reachable set
    /// @param dt Time in s with which the set is propagated
    void Propagate(float dt);
    /// Propagate the reachable set
    /// @param dt Time in s with which the set is propagated
    /// @param polyhedron The polyhedron to propagate.
    void Propagate(float dt, Polyhedron &polyhedron);

    /// Generate occupancies for future time intervals
    /// @param dt Time interval for which each occupancy is computed
    /// @param prediction_horizon Number of time steps into the future you want to
    /// generate the occupancies for
    /// @return Array of Polygons for the occupancy within each time interval
    std::vector<Polygon> ComputeFutureOccupancies(float dt, int prediction_horizon);
};

} // namespace cpp_occlusions

#endif