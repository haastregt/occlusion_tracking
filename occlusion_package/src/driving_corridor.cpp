#include "cpp_occlusions/utility.h"

#include "cpp_occlusions/driving_corridor.h"

#include <CGAL/draw_polygon_2.h>

namespace cpp_occlusions
{

DrivingCorridor::DrivingCorridor(Polygon lane_polygon, Polygon transformed_polygon, float max_triangulation_edge_length)
    : _original_polygon(lane_polygon), _source_shape(PolygonToShape(lane_polygon)),
      _target_shape(PolygonToShape(transformed_polygon)), _source_domain(Domain(_source_shape)),
      _target_domain(Domain(_target_shape)), _max_edge_length(IKernel::FT(max_triangulation_edge_length))
{
    assert(_source_shape.size() == _target_shape.size());

    const Point_range seeds = {IPoint2(0.5, 0.5)}; // = GetPointInside(); // TODO find point inside original polygon
    _source_domain.create(_max_edge_length, seeds);
    // seeds = GetPointInside(); // TODO find point inside target
    _target_domain.create(_max_edge_length, seeds);

    // We have to create Nef polyhedra that are slightly insetted to intersect the
    // inputs prior to a mapping, otherwise points could be outside the domain and
    // This results in the coordinates being mapped to 0,0, which leads to invalid
    // Polyhedra.
    Polygon inset_lane = InsetPolygon(lane_polygon, 0.01);
    Polygon inset_transformed = InsetPolygon(transformed_polygon, 0.01);
    InitialiseAsExtrudedPolygon<HalfedgeDS> extrude(inset_lane, std::pair<float, float>{-100, 100});

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

Polyhedron DrivingCorridor::TransformOriginalToMapped(Polyhedron &input_polyhedron)
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

Polyhedron DrivingCorridor::TransformMappedToOriginal(Polyhedron &input_polyhedron)
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

Point_range DrivingCorridor::TransformTargetToSource(Point_range coordinates)
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

Point_range DrivingCorridor::TransformSourceToTarget(Point_range coordinates)
{
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