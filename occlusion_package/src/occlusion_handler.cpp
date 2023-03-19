#include "../include/cpp_occlusions/occlusion_handler.h"

namespace cpp_occlusions {

OcclusionHandler::OcclusionHandler(PolygonListBinding driving_corridor_polygons, PolygonBinding initial_sensor_view, int init_time_step, ReachabilityParams params)
{
    
}

OcclusionHandler::~OcclusionHandler() {}

Polygon OcclusionHandler::Update(Polygon sensor_view)
{

    for(auto it = sensor_view.vertices_begin(); it!= sensor_view.vertices_end(); ++it){
        std::cout << *it << std::endl;
    }

    // check if the polygon is simple.
    std::cout << "The polygon is " <<
        (sensor_view.is_simple() ? "" : "not ") << "simple." << "\n";
    // check if the polygon is convex
    std::cout << "The polygon is " <<
        (sensor_view.is_convex() ? "" : "not ") << "convex." << "\n";

    CGAL::draw(sensor_view);
    return sensor_view;
}

std::list<PolygonListBinding> OcclusionHandler::GetReachableSets()
{
    // First element should be shadow shape (occupancy t=0), the rest the occupancies of future time steps
    std::cout << "This works! \n";
    std::list<PolygonListBinding> temp;
    return temp;
}

}