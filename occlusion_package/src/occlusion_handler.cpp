#include "../include/cpp_occlusions/occlusion_handler.h"

namespace cpp_occlusions {

OcclusionHandler::OcclusionHandler(std::list<Polygon> driving_corridor_polygons, Polygon initial_sensor_view, int init_time_step, ReachabilityParams params)
{
    for(auto it = driving_corridor_polygons.begin(); it!= driving_corridor_polygons.end(); ++it){
        CGAL::draw(*it);
    }
}

OcclusionHandler::~OcclusionHandler() {}

void OcclusionHandler::Update(Polygon sensor_view)
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
}

std::list<std::list<Polygon>> OcclusionHandler::GetReachableSets()
{
    // First element should be shadow shape (occupancy t=0), the rest the occupancies of future time steps
    
    Polygon p1;
    p1.push_back(Point2(0,0));
    p1.push_back(Point2(4,0));
    p1.push_back(Point2(4,4));
    p1.push_back(Point2(2,2));
    p1.push_back(Point2(0,4));

    Polygon p2;
    p2.push_back(Point2(4,0));
    p2.push_back(Point2(4,4));
    p2.push_back(Point2(2,2));

    Polygon p3;
    p3.push_back(Point2(2,2));
    p3.push_back(Point2(0,0));
    p3.push_back(Point2(4,0));

    std::list<Polygon> between;
    between.push_back(p1);
    between.push_back(p2);
    between.push_back(p3);
    std::list<std::list<Polygon>> temp;
    temp.push_back(between);
    temp.push_back(between);

    std::cout << "This works! \n";
    return temp;
}

}