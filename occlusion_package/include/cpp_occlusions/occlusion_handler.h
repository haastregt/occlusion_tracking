#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/draw_polygon_2.h>

#include <iostream>

#include "occluded_volume.h"

#ifndef OCCLUSION_HANDLER_H
#define OCCLUSION_HANDLER_H

// These reduce polygons to an iterator of points, so that pybind can do 
// typeconversion automatically
typedef std::list<std::vector<float>>                   PolygonBinding;
typedef std::list<std::list<std::vector<float>>>        PolygonListBinding; 

namespace cpp_occlusions {

class OcclusionHandler {

    private:
        
        int _time_step;
        ReachabilityParams _params;
        std::vector<OccludedVolume> _shadow_list;

    public:

        OcclusionHandler(PolygonListBinding driving_corridor_polygons, PolygonBinding initial_sensor_view, int init_time_step, ReachabilityParams params); 

        ~OcclusionHandler();

        /// Update the occlusions with new sensor view
        /// @param sensorview Polygon of the new FOV
        Polygon Update(Polygon sensorview);

        /// Get the reachable sets for future time intervals
        /// @return List of Polygon arrays for occupancy of each time interval
        std::list<PolygonListBinding> GetReachableSets();

};

}

#endif