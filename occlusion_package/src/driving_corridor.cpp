#include "cpp_occlusions/driving_corridor.h"

namespace cpp_occlusions
{

DrivingCorridor::DrivingCorridor(Polygon lane_polygon)
    : _original_polygon(lane_polygon), _source_shape(GetSourceShape(lane_polygon)),
      _target_shape(GetTargetShape(lane_polygon)), _source_domain(Domain(_source_shape)),
      _target_domain(Domain(_target_shape))
{
    assert(_source_shape.size() == coordinates.size());

    const Point_range seeds; // = GetPointInside(); // TODO find point inside original polygon
    _source_domain.create(_max_edge_length, seeds);

    // seeds = GetPointInside(); // TODO find point inside target
    _target_domain.create(_max_edge_length, seeds);
}

DrivingCorridor::~DrivingCorridor()
{
}

Point_range DrivingCorridor::GetSourceShape(Polygon lane)
{
    // Transform _original_polygon to _source_shape
    Point_range source_shape = {IPoint2(0, 0), IPoint2(0, -2), IPoint2(2, -2),
                                IPoint2(2, 2), IPoint2(-2, 2), IPoint2(-2, 0)};

    return source_shape;
}

Point_range DrivingCorridor::GetTargetShape(Polygon lane)
{
    // Logic for getting _target_shape from _source_shape
    Point_range target_shape = {IPoint2(0, 0), IPoint2(3, 0),  IPoint2(3, 2),
                                IPoint2(0, 2), IPoint2(-3, 2), IPoint2(-3, 0)};

    return target_shape;
}

void DrivingCorridor::TransformOriginalToMapped(Polyhedron &polyhedron)
{
    Point_range points;
    std::cout << "Original Points: ";
    for (auto vert_it = polyhedron.vertices_begin(); vert_it != polyhedron.vertices_end(); ++vert_it)
    {
        // Transform to IK since Harmonic Coordinates cant use exact coords
        Point2 exact_point(vert_it->point().x(), vert_it->point().y());
        points.push_back(to_inexact(exact_point));
        std::cout << to_inexact(exact_point) << ", ";
    }
    std::cout << std::endl;

    Point_range transformed_points = TransformSourceToTarget(points);

    std::cout << "Transformed Points: ";
    auto vert_it = polyhedron.vertices_begin();
    for (int i = 0; i < polyhedron.size_of_vertices(); i++)
    {
        IPoint2 inexact_point(transformed_points[i].x(), transformed_points[i].y());
        Point2 exact_point = to_exact(inexact_point);
        vert_it->point().x() = exact_point.x();
        vert_it->point().y() = exact_point.y();
        ++vert_it;
        std::cout << inexact_point << ", ";
    }
    std::cout << std::endl;
}

void DrivingCorridor::TransformMappedToOriginal(Polyhedron &polyhedron)
{
    Point_range points;
    for (auto vert_it = polyhedron.vertices_begin(); vert_it != polyhedron.vertices_end(); ++vert_it)
    {
        // Transform to IK since Harmonic Coordinates cant use exact coords
        Point2 exact_point(vert_it->point().x(), vert_it->point().y());
        points.push_back(to_inexact(exact_point));
    }

    Point_range transformed_points = TransformTargetToSource(points);

    auto vert_it = polyhedron.vertices_begin();
    for (int i = 0; i < polyhedron.size_of_vertices(); i++)
    {
        IPoint2 inexact_point(transformed_points[i].x(), transformed_points[i].y());
        Point2 exact_point = to_exact(inexact_point);
        vert_it->point().x() = exact_point.x();
        vert_it->point().y() = exact_point.y();
        ++vert_it;
    }
}

Point_range DrivingCorridor::TransformTargetToSource(Point_range coordinates)
{
    Harmonic_coordinates_2 harmonic_coordinates(coordinates, _target_domain);
    harmonic_coordinates.compute();
    std::vector<std::vector<double>> new_coordinates;

    harmonic_coordinates(std::back_inserter(new_coordinates));

    Point_range transformed_points;
    for (std::vector<double> coords : new_coordinates)
    {
        IKernel::FT x = IKernel::FT(0);
        IKernel::FT y = IKernel::FT(0);

        for (std::size_t i = 0; i < coords.size(); i++)
        {
            x += coords[i] * _source_shape[i].x();
            y += coords[i] * _source_shape[i].y();
        }
        transformed_points.push_back(IPoint2(x, y));
    }

    return transformed_points;
}

Point_range DrivingCorridor::TransformSourceToTarget(Point_range coordinates)
{
    Harmonic_coordinates_2 harmonic_coordinates(coordinates, _source_domain);
    harmonic_coordinates.compute();
    std::vector<std::vector<double>> new_coordinates;

    harmonic_coordinates(std::back_inserter(new_coordinates));

    Point_range transformed_points;
    for (std::vector<double> coords : new_coordinates)
    {
        double x = 0;
        double y = 0;

        for (std::size_t i = 0; i < coords.size(); i++)
        {
            x += coords[i] * _target_shape[i].x();
            y += coords[i] * _target_shape[i].y();
        }
        transformed_points.push_back(IPoint2(x, y));
    }

    return transformed_points;
}

} // namespace cpp_occlusions