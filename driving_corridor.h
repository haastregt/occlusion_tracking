#include "type_definitions.h"

#include <CGAL/Barycentric_coordinates_2/Delaunay_domain_2.h>
#include <CGAL/Barycentric_coordinates_2/Harmonic_coordinates_2.h>
#include <CGAL/Cartesian_converter.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>

namespace cpp_occlusions
{

using IKernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using IPoint2 = IKernel::Point_2;
using IPolyhedron = CGAL::Polyhedron_3<IKernel>;
using Point_range = std::vector<IPoint2>;
using Domain = CGAL::Barycentric_coordinates::Delaunay_domain_2<Point_range, IKernel>;
using Harmonic_coordinates_2 = CGAL::Barycentric_coordinates::Harmonic_coordinates_2<Point_range, Domain, IKernel>;

using IK_to_EK = CGAL::Cartesian_converter<Kernel, IKernel>;
using EK_to_IK = CGAL::Cartesian_converter<IKernel, Kernel>;

class DrivingCorridor
{
  private:
    Polygon _original_polygon;

    const std::vector<IPoint2> _source_shape = {IPoint2(1, 0), IPoint2(2, 0), IPoint2(3, 3), IPoint2(4, 0),
                                                IPoint2(5, 0), IPoint2(4, 3), IPoint2(4, 5), IPoint2(5, 4),
                                                IPoint2(5, 5), IPoint2(4, 6), IPoint2(2, 6), IPoint2(1, 5),
                                                IPoint2(1, 4), IPoint2(2, 5), IPoint2(2, 3)};
    const std::vector<IPoint2> _target_shape = {IPoint2(2, 0), IPoint2(3, 0), IPoint2(3, 3), IPoint2(3, 0),
                                                IPoint2(4, 0), IPoint2(4, 3), IPoint2(4, 5), IPoint2(5, 6),
                                                IPoint2(5, 7), IPoint2(4, 6), IPoint2(2, 6), IPoint2(1, 7),
                                                IPoint2(1, 6), IPoint2(2, 5), IPoint2(2, 3)};

    Domain _source_domain;
    Domain _target_domain;

    // This can be used to tune the quality vs computational time
    // Higher value - less accurate, faster computation
    const Kernel::FT _max_edge_length = Kernel::FT(1);

    void GenerateShapes();

    Point_range TransformSourceToTarget(Point_range coordinates);

    Point_range TransformTargetToSource(Point_range coordinates);

  public:
    DrivingCorridor(Polygon lane_polygon);

    ~DrivingCorridor();

    void TransformOriginalToMapped(Polyhedron &polyhedron);

    void TransformMappedToOriginal(Polyhedron &polyhedron);
};

} // namespace cpp_occlusions