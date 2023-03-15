#include "../include/cpp_occlusions/occlusion_handler.h"

#include <iostream>

namespace cpp_occlusions {

OcclusionHandler::OcclusionHandler(std::vector<Polygon> driving_corridor_polygons, Polygon initial_sensor_view, int init_time_step, ReachabilityParams params)
{
    
}

OcclusionHandler::~OcclusionHandler() {}

void OcclusionHandler::Update(Polygon sensor_view)
{

}

void OcclusionHandler::GetReachableSets()
{
    std::cout << "This works! \n";
}
}