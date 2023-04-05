#include "../include/cpp_occlusions/driving_corridor.h"

namespace cpp_occlusions
{

DrivingCorridor::DrivingCorridor(Polygon lane_polygon) : _original_polygon(lane_polygon)
{
    GenerateShapes();

    assert(_source_shape.size() == coordinates.size());

    const std::vector<Point2> seeds; // = GetPointInside(); // TODO find point inside original polygon
    _source_domain = Domain(_source_shape);
    _source_domain.create(_max_edge_length, seeds);

    // seeds = GetPointInside(); // TODO find point inside target
    _target_domain = Domain(_target_shape);
    _target_domain.create(_max_edge_length, seeds);
}

DrivingCorridor::~DrivingCorridor()
{
}

void DrivingCorridor::TransformOriginalToMapped(Polyhedron &polyhedron)
{
    // Transform to IK since Harmonic Coordinates cant use exact coords
    IPolyhedron inexact_polyhedron = EK_to_IK(polyhedron);

    Point_range points;
    for (auto vert_it = inexact_polyhedron.vertices_begin(); vert_it != inexact_polyhedron.vertices_end(); ++vert_it)
    {
        points.push_back(IPoint2(vert_it->x(), vert_it->y()));
    }

    Point_range transformed_points = TransformSourceToTarget(points);

    auto vert_it = inexact_polyhedron.vertices_begin();
    for (int i = 0; i < inexact_polyhedron.size_of_vertices(); i++)
    {
        vert_it->point() = IKernel::Point_3(transformed_points[i].x(), transformed_points[i].y(), vert_it->z());
        ++vert_it;
    }

    polyhedron = IK_to_EK(inexact_polyhedron);
}

void DrivingCorridor::TransformMappedToOriginal(Polyhedron &polyhedron)
{
    // Transform to IK since Harmonic Coordinates cant use exact coords
    IPolyhedron inexact_polyhedron = EK_to_IK(polyhedron);

    Point_range points;
    for (auto vert_it = inexact_polyhedron.vertices_begin(); vert_it != inexact_polyhedron.vertices_end(); ++vert_it)
    {
        points.push_back(IPoint2(vert_it->x(), vert_it->y()));
    }

    Point_range transformed_points = TransformTargetToSource(points);

    auto vert_it = inexact_polyhedron.vertices_begin();
    for (int i = 0; i < inexact_polyhedron.size_of_vertices(); i++)
    {
        vert_it->point() = IKernel::Point_3(transformed_points[i].x(), transformed_points[i].y(), vert_it->z());
        ++vert_it;
    }

    polyhedron = IK_to_EK(inexact_polyhedron);
}

void DrivingCorridor::GenerateShapes()
{
    // TODO: From polygon to point vector _source_shape
    // TODO: Logic for generating _target_shape from _source_shape
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