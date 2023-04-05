#include "type_definitions.h"

#include <CGAL/Barycentric_coordinates_2/Delaunay_domain_2.h>
#include <CGAL/Barycentric_coordinates_2/Harmonic_coordinates_2.h>
#include <CGAL/Cartesian_converter.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

namespace cpp_occlusions
{

using IKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using IPoint2 = IKernel::Point_2;
using Point_range = std::vector<IPoint2>;
using Domain = CGAL::Barycentric_coordinates::Delaunay_domain_2<Point_range, IKernel>;
using Harmonic_coordinates_2 = CGAL::Barycentric_coordinates::Harmonic_coordinates_2<Point_range, Domain, IKernel>;

using IK_to_EK = CGAL::Cartesian_converter<IKernel, Kernel>;
using EK_to_IK = CGAL::Cartesian_converter<Kernel, IKernel>;

class DrivingCorridor
{
  private:
    Polygon _original_polygon;

    const Point_range _source_shape;
    const Point_range _target_shape;

    Domain _source_domain;
    Domain _target_domain;

    // This can be used to tune the quality vs computational time
    // Higher value - less accurate, faster computation
    const IKernel::FT _max_edge_length = IKernel::FT(0.1);

    EK_to_IK to_inexact;
    IK_to_EK to_exact;

    Point_range GetSourceShape(Polygon lane);

    Point_range GetTargetShape(Polygon lane);

    Point_range TransformSourceToTarget(Point_range coordinates);

    Point_range TransformTargetToSource(Point_range coordinates);

  public:
    DrivingCorridor(Polygon lane_polygon);

    ~DrivingCorridor();

    void TransformOriginalToMapped(Polyhedron &polyhedron);

    void TransformMappedToOriginal(Polyhedron &polyhedron);
};

} // namespace cpp_occlusions