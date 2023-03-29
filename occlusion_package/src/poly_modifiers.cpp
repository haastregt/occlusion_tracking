#include "../include/cpp_occlusions/poly_modifiers.h"
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/draw_polygon_2.h>
#include <CGAL/draw_polyhedron.h>

namespace cpp_occlusions
{

Polygon ProjectXY(Polyhedron &polyhedron)
{
    bool add_point = true;

    Polygon polygon;
    std::list<Point> previous_points;
    std::vector<Polygon> ii;
    std::vector<Polygon_wh> oi;

    for (auto s = polyhedron.facets_begin(); s != polyhedron.facets_end(); ++s)
    {
        auto h = s->facet_begin(), he(h);

        polygon = Polygon();
        do
        {
            Point p1 = h->vertex()->point();
            if (previous_points.empty())
            {
                polygon.insert(polygon.vertices_end(), Point2(p1.x(), p1.y()));
                previous_points.push_back(p1);
            }
            else
            {
                for (Point previous_point : previous_points)
                {
                    if (p1.x() == previous_point.x() && p1.y() == previous_point.y())
                    {
                        add_point = false;
                        break;
                    }
                }

                if (add_point)
                {
                    polygon.insert(polygon.vertices_end(), Point2(p1.x(), p1.y()));
                    previous_points.push_back(p1);
                }
            }
            add_point = true;
        } while (++h != he);

        previous_points.clear();

        if (polygon.size() < 3)
        {
            // Then it would just be a point or line. Dont add this polygon to the list.
            // This happens for instance when one of the faces is perpendicular to the xy plane
            continue;
        }

        if (polygon.orientation() == CGAL::POSITIVE)
        {
            ii.push_back(polygon);
        }
    }
    CGAL::join(ii.begin(), ii.end(), std::back_inserter(oi));

    // The projection should result in just one polygon
    return oi.front().outer_boundary();
}

void DissolveCloseVertices(Polyhedron &polyhedron, float tolerance)
{
    for (auto vert_it = polyhedron.vertices_begin(); vert_it != polyhedron.vertices_end(); ++vert_it)
    {
        Polyhedron::Halfedge_around_vertex_circulator neighbour_it = vert_it->vertex_begin();
        do
        {
            if (CGAL::squared_distance(neighbour_it->opposite()->vertex()->point(), vert_it->point()) <
                tolerance * tolerance)
            {
                polyhedron.join_facet(neighbour_it->next());
                polyhedron.join_facet(neighbour_it->opposite()->next());
                polyhedron.join_vertex(neighbour_it);
            }
            break;

        } while (++neighbour_it != vert_it->vertex_begin());
    }
}

} // namespace cpp_occlusions