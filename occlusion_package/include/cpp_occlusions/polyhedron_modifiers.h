#include "type_definitions.h"
#include <CGAL/Polyhedron_incremental_builder_3.h>

#include <iostream>

#ifndef POLYHEDRON_MODIFIERS_H
#define POLYHEDRON_MODIFIERS_H

namespace cpp_occlusions
{

typedef Polyhedron::HalfedgeDS HalfedgeDS;

const int DIM = 3;

/// @brief  This Modifier will initialise a polyhedron based of an
/// extruded polygon.
template <class HDS> class InitialiseAsExtrudedPolygon : public CGAL::Modifier_base<HDS>
{
  public:
    Polygon polygon_ref;
    std::pair<float, float> bounds;

    InitialiseAsExtrudedPolygon(Polygon poly, std::pair<float, float> bnds) : polygon_ref(poly), bounds(bnds)
    {
    }

    void operator()(HDS &hds)
    {
        std::cout << "Inside operator function" << std::endl;
        Polygon polygon = Polygon(polygon_ref);
        std::cout << "Our polygon is: " << polygon << std::endl;
        CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);

        typedef typename HDS::Vertex Vertex;
        typedef typename Vertex::Point Point3;

        B.begin_surface(polygon.size(), polygon.size() + 2, polygon.size() * 3);

        // Make sure faces will have their normals pointing outwards
        if (!polygon.orientation() == CGAL::CLOCKWISE)
        {
            polygon.reverse_orientation();
        }

        std::cout << "We will now add points" << std::endl;
        for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it)
        {
            std::cout << "The problem is not dereferencing it anymore" << *it << std::endl;
            B.add_vertex(Point3(it->x(), it->y(), bounds.first));
            B.add_vertex(Point3(it->x(), it->y(), bounds.second));
        }

        // Bottom face
        B.begin_facet();
        for (int i = 0; i < polygon.size(); i++)
        {
            B.add_vertex_to_facet(i * 2);
        }
        B.end_facet();

        // Top face
        B.begin_facet();
        for (int i = polygon.size() - 1; i >= 0; i--)
        {
            B.add_vertex_to_facet(i * 2 + 1);
        }
        B.end_facet();

        // Side faces
        for (int i = 0; i < polygon.size() - 1; i++)
        {
            B.begin_facet();
            B.add_vertex_to_facet(2 * i);
            B.add_vertex_to_facet(2 * i + 1);
            B.add_vertex_to_facet(2 * i + 3);
            B.add_vertex_to_facet(2 * i + 2);
            B.end_facet();
        }
        B.begin_facet();
        B.add_vertex_to_facet(2 * (polygon.size() - 1));
        B.add_vertex_to_facet(2 * (polygon.size() - 1) + 1);
        B.add_vertex_to_facet(1);
        B.add_vertex_to_facet(0);
        B.end_facet();

        B.end_surface();
    }
};

// This might actually be possible to do with just std::transform() instead or using AffineTransformation_3 in CGAL
/// @brief This MModifier performs a linear map on the polyhedron.
/// Linear maps can be used to perform e.g. a skew operation with
/// [1 0 dt; 0 1 0; 0 0 1] or a projection with [1 0 0; 0 1 0; 0 0 0].
template <class HDS> class LinearMap : public CGAL::Modifier_base<HDS>
{
  public:
    std::array<std::array<float, DIM>, DIM> map;

    LinearMap(std::array<std::array<float, DIM>, DIM> m) : map(m)
    {
    }

    void operator()(HDS &hds)
    {
    }
};

/* These cant be in a header file because it gets defined multiple times due to multiple includes
/// @brief This modifier projects the polyhedron to the xy plane.
LinearMap<HalfedgeDS> project_xy_modifier((std::array<std::array<float, DIM>, DIM>){{{1, 0, 0}, {0, 1, 0}, {0, 0, 0}}});

/// @brief This modifier skews along x with the velocity
/// @param dt time step to know how much to skew
LinearMap<HalfedgeDS> skew_x_with_vel(float dt)
{
    std::array<std::array<float, DIM>, DIM> map = {{{1, 0, dt}, {0, 1, 0}, {0, 0, 1}}};
    LinearMap<HalfedgeDS> skew(map);
    return skew;
}
*/

} // namespace cpp_occlusions

#endif