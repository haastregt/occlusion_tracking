#include "cpp_occlusions/occluded_volume.h"

#ifndef OCCLUSION_HANDLER_H
#define OCCLUSION_HANDLER_H

namespace cpp_occlusions
{

class OcclusionHandler
{
  private:
    int _time_step;
    int _ID_allocator;

    double _update_time;
    double _prediction_time;
    int _num_updates;
    int _num_predictions;

    ReachabilityParams _params;

    std::list<std::list<OccludedVolume>> _shadow_list_by_corridor;

    // List of shadowlist by ID, shadowlist has a shadow for each timestep
    std::list<std::tuple<int, std::list<std::tuple<int, Polyhedron>>>> _shadow_saves;

  public:
    OcclusionHandler(std::list<Polygon> driving_corridor_polygons, std::list<Polygon> mapped_driving_corridor_polygons,
                     std::list<std::list<Polygon>> lanes_in_driving_corridors, Polygon initial_sensor_view,
                     int init_time_step, ReachabilityParams params);

    ~OcclusionHandler();

    /// Update the occlusions with new sensor view for the whole scene
    /// @param sensorview Polygon of the new FOV
    void Update(Polygon sensorview, int new_time_step);

    /// Get the reachable sets for future time intervals
    /// @return List of Polygon arrays for occupancy of each time interval
    std::list<std::list<Polygon>> GetReachableSets();

    /// @brief Finds the future occupancies for a known obstacle
    /// @param driving_corridor_poly The corresponding driving corridor of the obstacle
    /// @param lanes_in_corridor The lanes inside the corresponding driving corridor of the obstacle
    /// @param initial_set Initial occupancy to propagate
    /// @param velocity The velocity state of the obstacle
    /// @param params Reachability parameters
    /// @return A list of future occupancies as polygons
    static std::list<Polygon> PropagateKnownObstacle(Polygon driving_corridor_poly,
                                                     std::list<Polygon> lanes_in_corridor, Polygon initial_set,
                                                     double velocity, ReachabilityParams params);

    void SaveShadow(int ID, Polyhedron polyhedron);

    // This type is needed to convert to python types
    std::list<std::tuple<int, std::list<std::tuple<int, std::list<std::list<float>>>>>> ExportShadows();

    std::tuple<double, double> ExportComputationalTime();
};

} // namespace cpp_occlusions

#endif