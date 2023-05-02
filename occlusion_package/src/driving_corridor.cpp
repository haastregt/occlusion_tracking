#include "cpp_occlusions/driving_corridor.h"
#include "cpp_occlusions/utility.h"

namespace cpp_occlusions
{

DrivingCorridor::DrivingCorridor(Polygon lane_polygon, Polygon transformed_polygon, std::list<Polygon> dc_lanes, float max_triangulation_edge_length)
    : original_polygon(lane_polygon), _source_shape(PolygonToShape(lane_polygon)),
      _target_shape(PolygonToShape(transformed_polygon)), _source_domain(Domain(_source_shape)),
      _target_domain(Domain(_target_shape)), _max_edge_length(IKernel::FT(max_triangulation_edge_length)), lanes(dc_lanes)
{
    assert(_source_shape.size() == _target_shape.size());

    IPoint2 seed = GetPointInside(lane_polygon);
    const Point_range seeds_source = {seed};
    _source_domain.create(_max_edge_length, seeds_source);

    seed = GetPointInside(transformed_polygon);
    const Point_range seeds_target = {seed};
    _target_domain.create(_max_edge_length, seeds_target);

    // We have to create Nef polyhedra that are slightly insetted to intersect the
    // inputs prior to a mapping, otherwise points could be outside the domain and
    // This results in the coordinates being mapped to 0,0, which leads to invalid
    // Polyhedra.
    Polygon inset_lane = InsetPolygon(lane_polygon, 0.01);
    Polygon inset_transformed = InsetPolygon(transformed_polygon, 0.01);

    ExtrudeZ<HalfedgeDS> extrude(inset_lane, std::pair<float, float>{-100, 100});
    Polyhedron P = Polyhedron();
    P.delegate(extrude);
    _source_polyhedron = Nef_polyhedron(P);

    extrude.polygon = inset_transformed;
    P = Polyhedron();
    P.delegate(extrude);
    _target_polyhedron = Nef_polyhedron(P);
}

DrivingCorridor::~DrivingCorridor()
{
}

IPoint2 DrivingCorridor::GetPointInside(Polygon polygon)
{
    CGAL::Vector_2<Kernel> vec(Kernel::FT(0.1), Kernel::FT(0.1));
    Point2 vertex = *polygon.vertices_begin();
    for (int i = 0; i < 4; i++)
    {
        if (polygon.has_on_positive_side(vertex + vec))
        {
            return IPoint2(to_inexact(vertex + vec));
        }
        vec = vec.perpendicular(CGAL::COUNTERCLOCKWISE);
    }

    assert(false && "No point could be found inside the polygon. Check if the polygon is correct");
    return IPoint2();
}

Point_range DrivingCorridor::PolygonToShape(Polygon polygon)
{
    Point_range shape;

    for (auto vert_it = polygon.vertices_begin(); vert_it != polygon.vertices_end(); ++vert_it)
    {
        IPoint2 pt = to_inexact(Point2(vert_it->x(), vert_it->y()));
        shape.push_back(pt);
    }

    return shape;
}

Polyhedron DrivingCorridor::TransformOriginalToMapped(Polyhedron &input_polyhedron) const
{
    Polyhedron polyhedron;
    Nef_polyhedron nef_input(input_polyhedron);
    nef_input *= _source_polyhedron;
    nef_input.convert_to_polyhedron(polyhedron);

    Point_range points;
    for (auto vert_it = polyhedron.vertices_begin(); vert_it != polyhedron.vertices_end(); ++vert_it)
    {
        // Transform to IK since Harmonic Coordinates cant use exact coords
        Point2 exact_point(vert_it->point().x(), vert_it->point().y());
        points.push_back(to_inexact(exact_point));
    }

    Point_range transformed_points = TransformSourceToTarget(points);

    Polyhedron::Vertex_iterator vert_it = polyhedron.vertices_begin();
    for (int i = 0; i < polyhedron.size_of_vertices(); i++)
    {
        IPoint2 inexact_point(transformed_points[i].x(), transformed_points[i].y());
        Point2 exact_point = to_exact(inexact_point);
        vert_it->point() = Point(exact_point.x(), exact_point.y(), vert_it->point().z());
        ++vert_it;
    }

    assert(polyhedron.is_valid());
    return polyhedron;
}

Polyhedron DrivingCorridor::TransformMappedToOriginal(Polyhedron &input_polyhedron) const
{
    Polyhedron polyhedron;
    Nef_polyhedron nef_input(input_polyhedron);
    nef_input *= _target_polyhedron;
    nef_input.convert_to_polyhedron(polyhedron);

    Point_range points;
    for (auto vert_it = polyhedron.vertices_begin(); vert_it != polyhedron.vertices_end(); ++vert_it)
    {
        // Transform to IK since Harmonic Coordinates cant use exact coords
        Point2 exact_point(vert_it->point().x(), vert_it->point().y());
        points.push_back(to_inexact(exact_point));
    }
    Point_range transformed_points = TransformTargetToSource(points);
    Polyhedron::Vertex_iterator vert_it = polyhedron.vertices_begin();
    for (int i = 0; i < polyhedron.size_of_vertices(); i++)
    {
        IPoint2 inexact_point(transformed_points[i].x(), transformed_points[i].y());
        Point2 exact_point = to_exact(inexact_point);
        vert_it->point() = Point(exact_point.x(), exact_point.y(), vert_it->point().z());
        ++vert_it;
    }

    assert(polyhedron.is_valid());
    return polyhedron;
}

Point_range DrivingCorridor::TransformTargetToSource(Point_range coordinates) const
{
    Harmonic_coordinates_2 harmonic_coordinates(_target_shape, _target_domain);
    harmonic_coordinates.compute();

    Point_range transformed_points;
    for (IPoint2 coords : coordinates)
    {
        std::vector<double> hc;
        harmonic_coordinates(coords, std::back_inserter(hc));

        IKernel::FT x = IKernel::FT(0);
        IKernel::FT y = IKernel::FT(0);

        for (std::size_t i = 0; i < hc.size(); i++)
        {
            x += hc[i] * _source_shape[i].x();
            y += hc[i] * _source_shape[i].y();
        }

        transformed_points.push_back(IPoint2(x, y));
    }

    return transformed_points;
}

Point_range DrivingCorridor::TransformSourceToTarget(Point_range coordinates) const
{
    // TODO: This can probably be precomputed
    Harmonic_coordinates_2 harmonic_coordinates(_source_shape, _source_domain);
    harmonic_coordinates.compute();

    Point_range transformed_points;
    for (IPoint2 coords : coordinates)
    {
        std::vector<double> hc;
        harmonic_coordinates(coords, std::back_inserter(hc));

        IKernel::FT x = IKernel::FT(0);
        IKernel::FT y = IKernel::FT(0);

        for (std::size_t i = 0; i < hc.size(); i++)
        {
            x += hc[i] * _target_shape[i].x();
            y += hc[i] * _target_shape[i].y();
        }

        transformed_points.push_back(IPoint2(x, y));
    }

    return transformed_points;
}

} // namespace cpp_occlusions