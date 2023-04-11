#include "cpp_occlusions/type_definitions.h"

#include <CGAL/Polyhedron_incremental_builder_3.h>

#ifndef UTILITY_H
#define UTILITY_H

namespace cpp_occlusions
{

/// @brief  This Modifier will initialise a polyhedron based of an
/// extruded polygon.
template <class HDS> class InitialiseAsExtrudedPolygon : public CGAL::Modifier_base<HDS>
{
  public:
    Polygon polygon;
    std::pair<float, float> bounds;

    InitialiseAsExtrudedPolygon(Polygon poly, std::pair<float, float> bnds) : polygon(poly), bounds(bnds)
    {
    }

    void operator()(HDS &hds)
    {
        typedef typename HDS::Vertex::Point Point3;

        CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);

        // Make sure faces will have their normals pointing outwards
        if (!polygon.orientation() == CGAL::CLOCKWISE)
        {
            polygon.reverse_orientation();
        }

        B.begin_surface(polygon.size(), polygon.size() + 2, polygon.size() * 3);

        for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it)
        {
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

// TODO: See if this can be optimized (e.g. only take faces with normal pointing upwards,
// or doing a projection with linear transform and then intersect with a plane at z = 0)
Polygon ProjectXY(Polyhedron &polyhedron);

void DissolveCloseVertices(Polyhedron &polyhedron, float tolerance);

Polygon InsetPolygon(Polygon &polygon, float inset_distance);

void SimplifyPolyhedron(Polyhedron& poly, float precision);

} // namespace cpp_occlusions

#endif