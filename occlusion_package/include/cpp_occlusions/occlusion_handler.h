#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polyhedron.h>

#include <iostream>

#include "occluded_volume.h"

#ifndef OCCLUSION_HANDLER_H
#define OCCLUSION_HANDLER_H

namespace cpp_occlusions
{

class OcclusionHandler
{
  private:
    int _time_step;
    ReachabilityParams _params;
    std::list<OccludedVolume> _shadow_list;
    std::list<Polygon> _driving_corridors;

  public:
    OcclusionHandler(std::list<Polygon> driving_corridor_polygons, Polygon initial_sensor_view, int init_time_step,
                     ReachabilityParams params);

    ~OcclusionHandler();

    /// Update the occlusions with new sensor view for the whole scene
    /// @param sensorview Polygon of the new FOV
    void Update(Polygon sensorview, float new_time_step);

    /// Get the reachable sets for future time intervals
    /// @return List of Polygon arrays for occupancy of each time interval
    std::list<std::list<Polygon>> GetReachableSets();
};

} // namespace cpp_occlusions

#endif