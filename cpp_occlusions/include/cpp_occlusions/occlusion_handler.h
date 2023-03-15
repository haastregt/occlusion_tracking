#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>

#include "occluded_volume.h"

#ifndef OCCLUSION_HANDLER_H
#define OCCLUSION_HANDLER_H

namespace cpp_occlusions {

class OcclusionHandler {

    private:
        
        int _time_step;
        ReachabilityParams _params;
        std::vector<OccludedVolume> _shadow_list;

    public:

        OcclusionHandler(std::vector<Polygon> driving_corridor_polygons, Polygon initial_sensor_view, int init_time_step, ReachabilityParams params); 

        ~OcclusionHandler();

        void Update(Polygon sensorview);

        void GetReachableSets();
        // This should return something eventually

};

}

#endif