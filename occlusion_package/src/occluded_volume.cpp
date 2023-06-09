#include "cpp_occlusions/occluded_volume.h"
#include "cpp_occlusions/utility.h"

#include <cmath>

#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/minkowski_sum_2.h>


namespace cpp_occlusions
{

OccludedVolume::OccludedVolume(Polyhedron initial_polyhedron, std::shared_ptr<DrivingCorridor> driving_corridor,
                               const ReachabilityParams params, int ID)
    : _shadow_polyhedron(initial_polyhedron), _driving_corridor(driving_corridor), _params(params), _ID(ID)
{
}

OccludedVolume::~OccludedVolume()
{
}

Nef_polyhedron OccludedVolume::VelocityAbstraction(float dt, Polyhedron polyhedron)
{
    int n;
    float R;
    float phi;

    n = 2; // Number of vertices to approximate the circle section
    phi = _params.phi / 180 * M_PI;
    // Use apothem to make sure we have an over-approximation of the circle section
    R = _params.vmax * dt / cos(M_PI / (n * (M_PI / phi))); 

    Polygon sum;
    sum.push_back(Point2(0, 0));
    for (int i = 0; i <= n; i++)
    {
        float angle = i * 2 * phi / n - phi;
        sum.push_back(Point2(R * cos(angle), R * sin(angle)));
    }

    if (_params.requires_mapping)
    {
        polyhedron = _driving_corridor->TransformOriginalToMapped(polyhedron);
    }

    Polygon xy = ProjectXY(polyhedron);

    xy = CGAL::minkowski_sum_2(xy, sum).outer_boundary();

    ExtrudeZ<HalfedgeDS> extrude(xy, std::pair<float, float>{-9999, 9999});
    polyhedron = Polyhedron();
    polyhedron.delegate(extrude);

    if (_params.requires_mapping)
    {
        polyhedron = _driving_corridor->TransformMappedToOriginal(polyhedron);
    }

    return Nef_polyhedron(polyhedron);
}

Nef_polyhedron OccludedVolume::VelocityAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron)
{
    return VelocityAbstraction(time_interval.second, polyhedron);
}

Nef_polyhedron OccludedVolume::AccelerationAbstraction(float dt, Polyhedron polyhedron)
{
    CGAL::Aff_transformation_2<Kernel> A(Kernel::RT(1), Kernel::RT(dt), Kernel::RT(0), Kernel::RT(1));

    Polygon sum;
    float xu = 0.5 * pow(dt, 2) * _params.amax;
    float xl = 0.5 * pow(dt, 2) * _params.amin;
    float vu = dt * _params.amax;
    float vl = dt * _params.amin;
    sum.push_back(Point2(xu, vu));
    sum.push_back(Point2(xl, vu));
    sum.push_back(Point2(xl, vl));
    sum.push_back(Point2(xu, vl));

    if (_params.requires_mapping)
    {
        polyhedron = _driving_corridor->TransformOriginalToMapped(polyhedron);
    }

    Polygon xv = ProjectXZ(polyhedron);

    xv = CGAL::transform(A, xv);
    xv = CGAL::minkowski_sum_2(xv, sum).outer_boundary();

    ExtrudeY<HalfedgeDS> extrude(xv, std::pair<float, float>{-9999, 9999});
    polyhedron = Polyhedron();
    polyhedron.delegate(extrude);

    if (_params.requires_mapping)
    {
        polyhedron = _driving_corridor->TransformMappedToOriginal(polyhedron);
    }

    return Nef_polyhedron(polyhedron);
}

Nef_polyhedron OccludedVolume::AccelerationAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron)
{
    float t_lower = time_interval.first;
    float t_upper = time_interval.second;

    CGAL::Aff_transformation_2<Kernel> A_lower(Kernel::RT(1), Kernel::RT(t_lower), Kernel::RT(0), Kernel::RT(1));
    CGAL::Aff_transformation_2<Kernel> A_upper(Kernel::RT(1), Kernel::RT(t_upper), Kernel::RT(0), Kernel::RT(1));

    Polygon sum;
    float xu = 0.5 * pow(t_upper, 2) * _params.amax;
    float xl = 0.5 * pow(t_upper, 2) * _params.amin;
    float vu = t_upper * _params.amax;
    float vl = t_upper * _params.amin;
    sum.push_back(Point2(xu, vu));
    sum.push_back(Point2(xl, vu));
    sum.push_back(Point2(xl, vl));
    sum.push_back(Point2(xu, vl));

    if (_params.requires_mapping)
    {
        polyhedron = _driving_corridor->TransformOriginalToMapped(polyhedron);
    }

    Polygon xv = ProjectXZ(polyhedron);

    Polygon xv_lower = CGAL::transform(A_lower, xv);
    Polygon xv_upper = CGAL::transform(A_upper, xv);
    Polygon combined;
    combined.insert(combined.vertices_end(), xv_lower.vertices_begin(), xv_lower.vertices_end());
    combined.insert(combined.vertices_end(), xv_upper.vertices_begin(), xv_upper.vertices_end());

    std::vector<Point2> result;
    CGAL::convex_hull_2(combined.vertices_begin(), combined.vertices_end(), std::back_inserter(result));
    xv = Polygon();
    for (Point2 point : result)
    {
        xv.push_back(point);
    }

    xv = CGAL::minkowski_sum_2(xv, sum).outer_boundary();

    ExtrudeY<HalfedgeDS> extrude(xv, std::pair<float, float>{-9999, 9999});
    polyhedron = Polyhedron();
    polyhedron.delegate(extrude);

    if (_params.requires_mapping)
    {
        polyhedron = _driving_corridor->TransformMappedToOriginal(polyhedron);
    }

    return Nef_polyhedron(polyhedron);
}

Nef_polyhedron OccludedVolume::ExplicitNoReversingAbstraction(float x_prev)
{
    // Construct positive side of previous x value and map it to lane shape
    Polygon rect;
    if (x_prev == 0)
    {
        rect.push_back(Point2(x_prev, -9999));
        rect.push_back(Point2(9999, -9999));
        rect.push_back(Point2(9999, 9999));
        rect.push_back(Point2(x_prev, 9999));
    }
    else
    {
        rect.push_back(Point2(x_prev, -9999));
        rect.push_back(Point2(x_prev + 9999, -9999));
        rect.push_back(Point2(x_prev + 9999, 9999));
        rect.push_back(Point2(x_prev, 9999));
    }

    ExtrudeZ<HalfedgeDS> extrude(rect, std::pair<float, float>{-9999, 9999});
    Polyhedron positive_space;
    positive_space.delegate(extrude);

    if (_params.requires_mapping)
    {
        positive_space = _driving_corridor->TransformMappedToOriginal(positive_space);
    }

    return Nef_polyhedron(positive_space);
}

double OccludedVolume::GetMinX(Polyhedron polyhedron)
{
    if (_params.requires_mapping)
    {
        polyhedron = _driving_corridor->TransformOriginalToMapped(polyhedron);
    }

    double x_min = 9999;
    for (auto it = polyhedron.points_begin(); it != polyhedron.points_end(); ++it)
    {
        double x = CGAL::to_double(it->x());
        x_min = std::min(x_min, x);
    }

    return x_min;
}

double OccludedVolume::GetMinX(Polygon polygon)
{
    // This does not work if mapping is required because I dont have a function to map polygons
    // Instead, use polyhedra overload (you can just extrude polygon)
    double x_min = 9999;
    for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it)
    {
        double x = CGAL::to_double(it->x());
        x_min = std::min(x_min, x);
    }

    return x_min;
}

Nef_polyhedron OccludedVolume::GetLanes()
{
    Polygon_wh occupied_lanes = Polygon_wh();
    Polygon shadow_projection = ProjectXY(_shadow_polyhedron);
    for (Polygon lane : _driving_corridor->lanes)
    {
        Polygon inset_lane = InsetPolygon(lane, 0.3);
        if (CGAL::do_intersect(shadow_projection, inset_lane))
        {
            if (occupied_lanes.outer_boundary().is_empty())
            {
                occupied_lanes = Polygon_wh(lane);
                continue;
            }
            else
            {
                CGAL::join(lane, occupied_lanes.outer_boundary(), occupied_lanes);
            }
        }
    }
    if (occupied_lanes.outer_boundary().is_empty())
    {
        return Nef_polyhedron::EMPTY;
    }
    else
    {
        ExtrudeZ<HalfedgeDS> extrude(occupied_lanes.outer_boundary(),
                                     std::pair<float, float>{_params.vmin, _params.vmax});
        Polyhedron extruded_lanes;
        extruded_lanes.delegate(extrude);
        return Nef_polyhedron(extruded_lanes);
    }
}

std::list<OccludedVolume> OccludedVolume::Propagate(float dt, Polygon &sensor_view)
{
    Nef_polyhedron new_shadow(Nef_polyhedron::COMPLETE);

    if (_params.velocity_tracking_enabled)
    {
        Nef_polyhedron acceleration_abstraction = AccelerationAbstraction(dt, _shadow_polyhedron);
        new_shadow *= acceleration_abstraction;
    }

    Nef_polyhedron velocity_abstraction = VelocityAbstraction(dt, _shadow_polyhedron);
    new_shadow *= velocity_abstraction;

    std::list<CGAL::Polygon_with_holes_2<Kernel>> output_list;
    ExtrudeZ<HalfedgeDS> extrude(Polygon(), std::pair<float, float>{_params.vmin, _params.vmax});
    CGAL::difference(_driving_corridor->original_polygon, sensor_view, std::back_inserter(output_list));
    
    std::list<OccludedVolume> new_shadow_list;
    for (CGAL::Polygon_with_holes_2<Kernel> diff : output_list)
    {
        Nef_polyhedron copy = Nef_polyhedron(new_shadow);

        extrude.polygon = diff.outer_boundary();
        Polyhedron P;
        P.delegate(extrude);

        copy *= Nef_polyhedron(P);
        copy.convert_to_polyhedron(P);

        if (P.is_empty()) {continue;}
        if (abs(ProjectXY(P).area()) < _params.min_shadow_volume) {continue;}
        // SimplifyPolyhedron(P, _params.simplification_precision);
        new_shadow_list.push_back(OccludedVolume(P, _driving_corridor, _params, _ID));
    }

    return new_shadow_list;
}

std::list<Polygon> OccludedVolume::ComputeFutureOccupancies()
{
    std::list<Polygon> occupancy_set;
    Polygon shadow_xy_proj = ProjectXY(_shadow_polyhedron);
    occupancy_set.push_back(shadow_xy_proj);

    // Nef_polyhedron occupied_lanes = GetLanes();
    ExtrudeZ<HalfedgeDS> extrude(Polygon(),
                                 std::pair<float, float>{_params.vmin, _params.vmax});

    double min_x = 0;
    int num_predictions = _params.prediction_horizon / _params.prediction_interval;
    for (int i = 1; i <= num_predictions; i++)
    {
        std::pair<float, float> interval(_params.dt * _params.prediction_interval * (i - 1),
                                         _params.dt * _params.prediction_interval * i);

        Polygon_wh complete_occupancy;

        for (Polygon lane : _driving_corridor->lanes)
        {
            Polygon inset_lane = InsetPolygon(lane, 0.3);

            Polyhedron lane_polyhedron;
            extrude.polygon = lane;
            lane_polyhedron.delegate(extrude);
            Nef_polyhedron extruded_lane = Nef_polyhedron(lane_polyhedron);

            // std::list<Polygon_wh> intersections;
            // CGAL::intersection(inset_lane, shadow_xy_proj, std::back_inserter(intersections));
            if (!CGAL::do_intersect(inset_lane, shadow_xy_proj))
            {
                continue;
            }
            Nef_polyhedron shadow_in_lane = extruded_lane*Nef_polyhedron(_shadow_polyhedron);
            Polyhedron extruded_intersection;
            shadow_in_lane.convert_to_polyhedron(extruded_intersection);

            Nef_polyhedron occupancy(Nef_polyhedron::COMPLETE);

            if (_params.velocity_tracking_enabled)
            {
                Nef_polyhedron acceleration_abstraction = AccelerationAbstraction(interval, extruded_intersection);
                occupancy *= acceleration_abstraction;
            }

            Nef_polyhedron velocity_abstraction = VelocityAbstraction(interval, extruded_intersection);
            occupancy *= velocity_abstraction;

            Nef_polyhedron no_reversing = ExplicitNoReversingAbstraction(min_x);
            occupancy *= no_reversing;

            occupancy *= extruded_lane;
            
            if (occupancy == Nef_polyhedron::EMPTY)
            {
                continue;
            }

            Polyhedron P;
            occupancy.convert_to_Polyhedron(P);
            
            if (complete_occupancy.outer_boundary().is_empty())
            {
                complete_occupancy = Polygon_wh(ProjectXY(P));
            }
            else
            {
                CGAL::join(ProjectXY(P), complete_occupancy.outer_boundary(), complete_occupancy);
            }
        }

        if (complete_occupancy.outer_boundary().is_empty())
        {
            break;
        }

        min_x = GetMinX(complete_occupancy.outer_boundary());
        occupancy_set.push_back(complete_occupancy.outer_boundary());
    }

    return occupancy_set;
}

Polyhedron OccludedVolume::GetPolyhedron()
{
    return _shadow_polyhedron;
}

std::shared_ptr<DrivingCorridor> OccludedVolume::GetDrivingCorridor()
{
    return _driving_corridor;
}

} // namespace cpp_occlusions