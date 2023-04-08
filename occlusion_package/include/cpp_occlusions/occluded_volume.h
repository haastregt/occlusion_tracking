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

    DrivingCorridor *_driving_corridor;

    Polyhedron _shadow_polyhedron;

    /// @brief Applies the velocity reachability abstraction: R_Mvel(P) = { [x, y, v] | [x, y] in projxy(R_Macc(P)) +
    /// Circle(Vmax*dt), x in vbounds}
    /// @param dt The time step into the future for which the reachability abstraction is computed
    /// @param polyhedron The polyhedron that the abstraction is applied to
    Nef_polyhedron VelocityAbstraction(float dt, Polyhedron polyhedron);

    /// @brief Applies the velocity reachability abstraction: R_Mvel(P) = { [x, y, v] | [x, y] in projxy(R_Macc(P)) +
    /// Circle(Vmax*dt), x in vbounds}
    /// @param time_interval The time interval over which the reachability abstraction is computed
    /// @param polyhedron The polyhedron that the abstraction is applied to
    Nef_polyhedron VelocityAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron);

    /// @brief  Applies the acceleration reachability abstraction: R_Macc(P) = { A*P + B*U_abounds }
    /// @param dt The time step into the future for which the reachability abstraction is computed
    /// @param polyhedron The polyhedron that the abstraction is applied to
    Nef_polyhedron AccelerationAbstraction(float dt, Polyhedron polyhedron);

    /// @brief  Applies the acceleration reachability abstraction: R_Macc(P) = { A*P + B*U_abounds }
    /// @param time_interval The time interval over which the reachability abstraction is computed
    /// @param polyhedron The polyhedron that the abstraction is applied to
    Nef_polyhedron AccelerationAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron);

  public:
    OccludedVolume(Polyhedron initial_polyhedron, DrivingCorridor *driving_corridor, const ReachabilityParams params);

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
    DrivingCorridor *GetDrivingCorridor();
};

} // namespace cpp_occlusions

#endif