#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polyhedron_3.h>

#ifndef TYPE_DEFINITIONS_H
#define TYPE_DEFINITIONS_H

namespace cpp_occlusions
{

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Point_2<Kernel> Point2;
typedef CGAL::Point_3<Kernel> Point;
typedef CGAL::Polygon_2<Kernel> Polygon;
typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_wh;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;

struct ReachabilityParams
{
    float vmin;
    float vmax;
    float amin;
    float amax;
    float phi;

    float dt;
    int prediction_interval;
    int prediction_horizon;
    float min_shadow_volume;
    float mapping_quality;
    float simplification_precision;
    bool requires_mapping;

    bool velocity_tracking_enabled;
    bool export_shadows;
};

} // namespace cpp_occlusions

#endif