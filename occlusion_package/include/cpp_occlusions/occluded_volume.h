#include "cpp_occlusions/driving_corridor.h"
#include "cpp_occlusions/type_definitions.h"

#ifndef OCCLUDED_VOLUME_H
#define OCCLUDED_VOLUME_H

namespace cpp_occlusions
{

class OccludedVolume
{
  private:
    const ReachabilityParams _params;

    std::shared_ptr<DrivingCorridor> _driving_corridor;

    Polyhedron _shadow_polyhedron;

    /// @brief Computes the reachability of the velocity abstraction: R_Mvel(P)
    /// @param dt The time step into the future for which the reachability is computed
    /// @param polyhedron The initial set of states
    Nef_polyhedron VelocityAbstraction(float dt, Polyhedron polyhedron);

    /// @brief Computes the reachability of the velocity abstraction: R_Mvel(P)
    /// @param time_interval The time interval over which the reachability is computed
    /// @param polyhedron The initial set of states
    Nef_polyhedron VelocityAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron);

    /// @brief Computes the reachability of the acceleration abstraction: R_Macc(P)
    /// @param dt The time step into the future for which the reachability is computed
    /// @param polyhedron The initial set of states
    Nef_polyhedron AccelerationAbstraction(float dt, Polyhedron polyhedron);

    /// @brief Computes the reachability of the acceleration abstraction: R_Macc(P)
    /// @param time_interval The time interval over which the reachability is computed
    /// @param polyhedron The initial set of states
    Nef_polyhedron AccelerationAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron);

    /// @brief Computes polyhedron only containing space on the positive side of the previous minimal x
    /// @param x_prev The minimal value of x of the previously timestep
    Nef_polyhedron ExplicitNoReversingAbstraction(float x_prev);

    /// @brief Find which lanes the occluded volume is covering
    /// @return A polyhedron which is an extrusion of the union of lanes covered by the occlusion
    Nef_polyhedron GetLanes();

    /// @brief Gets the minimal x value in the mapped frame
    /// @param polyhedron the set to analyze
    /// @return the minimal x of the set
    double GetMinX(Polyhedron polyhedron);

  public:
    const int _ID;

    OccludedVolume(Polyhedron initial_polyhedron, std::shared_ptr<DrivingCorridor> driving_corridor,
                   const ReachabilityParams params, int ID);

    ~OccludedVolume();

    /// Propagate the reachable set
    /// @param dt Time in s with which the set is propagated
    /// @param sensor_view The sensor view of this time step
    std::list<OccludedVolume> Propagate(float dt, Polygon &sensor_view);

    /// Generate occupancies for future time intervals
    /// @param dt Time interval for which each occupancy is computed
    /// @param prediction_horizon Number of time steps into the future you want to
    /// generate the occupancies for
    /// @return Array of Polygons for the occupancy within each time interval. The first polygon is the shadow itself.
    std::list<Polygon> ComputeFutureOccupancies();

    /// @brief Getter for the private _shadow_polyhedron variable
    /// @return The shadow polyhedron
    Polyhedron GetPolyhedron();

    /// @brief Getter for the private _driving_corridor variable
    /// @return The driving corridor for the shadow
    std::shared_ptr<DrivingCorridor> GetDrivingCorridor();
};

} // namespace cpp_occlusions

#endif