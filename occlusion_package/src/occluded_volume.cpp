#include "cpp_occlusions/occluded_volume.h"
#include "cpp_occlusions/utility.h"

#include <cmath>

#include <CGAL/Aff_transformation_2.h>
#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/minkowski_sum_3.h>

#include <CGAL/draw_nef_3.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polyhedron.h>

namespace cpp_occlusions
{

OccludedVolume::OccludedVolume(Polyhedron initial_polyhedron, DrivingCorridor *driving_corridor,
                               const ReachabilityParams params, int ID)
    : _shadow_polyhedron(initial_polyhedron), _driving_corridor(driving_corridor), _params(params), _ID(ID)
{
}

OccludedVolume::~OccludedVolume()
{
}

Nef_polyhedron OccludedVolume::VelocityAbstraction(float dt, Polyhedron polyhedron, bool no_lateral_expansion /*=false*/)
{
    int n;
    float R;
    float phi;
    if (no_lateral_expansion)
    {
        // Circle section with ~0 angle
        n = 2;
        R = _params.vmax * dt; 
        phi = 0.00001;
    }
    else
    {
        n = 2; // Number of vertices to approximate the circle section
        phi = _params.phi / 180 * M_PI;
        R = _params.vmax * dt / cos(M_PI / (n*(M_PI/phi))); // Use apothem to make sure we have an over-approximation of the circle section
    }

    Polygon sum;
    sum.push_back(Point2(0,0));
    for (int i = 0; i <= n; i++)
    {
        float angle = i * 2*phi / n - phi;
        sum.push_back(Point2(R * cos(angle), R * sin(angle)));
    }

    if (_params.requires_mapping)
    {
        polyhedron = _driving_corridor->TransformOriginalToMapped(polyhedron);
        CGAL::convex_hull_3(polyhedron.points_begin(), polyhedron.points_end(), polyhedron);
    }

    Polygon xy = ProjectXY(polyhedron);

    xy = CGAL::minkowski_sum_2(xy, sum).outer_boundary();

    ExtrudeZ<HalfedgeDS> extrude(xy, std::pair<float, float>{-9999, 9999});
    polyhedron = Polyhedron();
    polyhedron.delegate(extrude);

    if (_params.requires_mapping)
    {
        polyhedron = _driving_corridor->TransformMappedToOriginal(polyhedron);
        CGAL::convex_hull_3(polyhedron.points_begin(), polyhedron.points_end(), polyhedron);
    }

    return Nef_polyhedron(polyhedron);
}

Nef_polyhedron OccludedVolume::VelocityAbstraction(std::pair<float, float> time_interval, Polyhedron polyhedron, bool no_lateral_expansion /*=false*/)
{
    return VelocityAbstraction(time_interval.second, polyhedron, no_lateral_expansion);
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
        CGAL::convex_hull_3(polyhedron.points_begin(), polyhedron.points_end(), polyhedron);
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
        CGAL::convex_hull_3(polyhedron.points_begin(), polyhedron.points_end(), polyhedron);
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
        CGAL::convex_hull_3(polyhedron.points_begin(), polyhedron.points_end(), polyhedron);
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
        CGAL::convex_hull_3(polyhedron.points_begin(), polyhedron.points_end(), polyhedron);
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
        CGAL::convex_hull_3(positive_space.points_begin(), positive_space.points_end(), positive_space);
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
    for(auto it = polyhedron.points_begin(); it != polyhedron.points_end(); ++it)
    {
        double x = CGAL::to_double(it->x());
        x_min = std::min(x_min, x);
    }

    return x_min;
}


std::list<OccludedVolume> OccludedVolume::Propagate(float dt, Polygon &sensor_view)
{
    std::list<OccludedVolume> new_shadow_list;

    Nef_polyhedron new_shadow(Nef_polyhedron::COMPLETE);
    Polyhedron P;
    CGAL::Surface_mesh<Nef_polyhedron::Point_3> surface_mesh;

    if (_params.velocity_tracking_enabled)
    {
        Nef_polyhedron acceleration_abstraction = AccelerationAbstraction(dt, _shadow_polyhedron);
        new_shadow *= acceleration_abstraction;
    }

    Nef_polyhedron velocity_abstraction = VelocityAbstraction(dt, _shadow_polyhedron);
    new_shadow *= velocity_abstraction;

    new_shadow.convert_to_polyhedron(P);
    SimplifyPolyhedron(P, _params.simplification_precision);
    new_shadow = Nef_polyhedron(P);

    Nef_polyhedron copy;
    std::list<CGAL::Polygon_with_holes_2<Kernel>> output_list;
    ExtrudeZ<HalfedgeDS> extrude(Polygon(), std::pair<float, float>{_params.vmin, _params.vmax});
    CGAL::difference(_driving_corridor->original_polygon, sensor_view, std::back_inserter(output_list));

    for (CGAL::Polygon_with_holes_2<Kernel> diff : output_list)
    {
        assert(diff.outer_boundary().is_simple() && "Polygon has a self-intersection!");
        assert((diff.outer_boundary().size() > 2) && "Polygon should have at least three points");

        copy = Nef_polyhedron(new_shadow);
        P = Polyhedron();

        extrude.polygon = diff.outer_boundary();
        P.delegate(extrude);
        assert(P.is_closed() && "Polyhedra should be closed in order for conversion to Nef");
        copy *= Nef_polyhedron(P);

        assert(copy.is_simple() && "Nef_Polyhedra should be simple in order for conversion to normal polyhedra");

        // We check if the minimal volume is above a treshhold to avoid small shadows due to numerical rounding
        // TODO: Maybe not necessary since we use exact construction kernel
        surface_mesh = CGAL::Surface_mesh<Nef_polyhedron::Point_3>();
        CGAL::convert_nef_polyhedron_to_polygon_mesh(copy, surface_mesh, true);

        if (CGAL::Polygon_mesh_processing::volume(surface_mesh) > _params.min_shadow_volume)
        {
            P = Polyhedron();
            copy.convert_to_polyhedron(P);

            // DissolveCloseVertices(P, 0.01);
            new_shadow_list.push_back(OccludedVolume(P, _driving_corridor, _params, _ID));
        }
    }

    return new_shadow_list;
}

std::list<Polygon> OccludedVolume::ComputeFutureOccupancies()
{
    std::list<Polygon> occupancy_set;
    occupancy_set.push_back(ProjectXY(_shadow_polyhedron));

    Polyhedron P;
    ExtrudeZ<HalfedgeDS> extrude(_driving_corridor->original_polygon,
                                 std::pair<float, float>{_params.vmin, _params.vmax});
    P.delegate(extrude);
    Nef_polyhedron nef_road(P);

    double min_x = 0;
    int num_predictions = _params.prediction_horizon / _params.prediction_interval;
    for (int i = 1; i <= num_predictions; i++)
    {
        std::pair<float, float> interval(_params.dt*_params.prediction_interval*(i-1), _params.dt*_params.prediction_interval*i);
        
        Nef_polyhedron occupancy(Nef_polyhedron::COMPLETE);

        if (_params.velocity_tracking_enabled)
        {
            Nef_polyhedron acceleration_abstraction =
                AccelerationAbstraction(interval, _shadow_polyhedron);
            occupancy *= acceleration_abstraction;
        }

        Nef_polyhedron velocity_abstraction =
            VelocityAbstraction(interval, _shadow_polyhedron, true);
        occupancy *= velocity_abstraction;

        Nef_polyhedron no_reversing = ExplicitNoReversingAbstraction(min_x);
        occupancy *= no_reversing;
        
        occupancy *= nef_road;

        P = Polyhedron();
        occupancy.convert_to_Polyhedron(P);

        if(occupancy == Nef_polyhedron::EMPTY)
        {
            break;
        }

        min_x = GetMinX(P);
        occupancy_set.push_back(ProjectXY(P));
    }

    return occupancy_set;
}

Polyhedron OccludedVolume::GetPolyhedron()
{
    return _shadow_polyhedron;
}

DrivingCorridor *OccludedVolume::GetDrivingCorridor()
{
    return _driving_corridor;
}

} // namespace cpp_occlusions