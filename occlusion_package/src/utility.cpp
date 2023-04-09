#include "cpp_occlusions/utility.h"

#include <CGAL/Boolean_set_operations_2.h>

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

Polygon InsetPolygon(Polygon &polygon, float inset_distance)
{
    Polygon inset_polygon;
    for (auto vertex = polygon.vertices_begin(); vertex != polygon.vertices_end(); ++vertex)
    {
        auto prev_vertex = (vertex == polygon.vertices_begin()) ? polygon.vertices_end() - 1 : vertex - 1;
        auto next_vertex = (vertex == polygon.vertices_end() - 1) ? polygon.vertices_begin() : vertex + 1;

        CGAL::Vector_2<Kernel> prev_edge = *vertex - *prev_vertex;
        CGAL::Vector_2<Kernel> next_edge = *next_vertex - *vertex;
        double prev_edge_length = CGAL::sqrt(CGAL::to_double(prev_edge.squared_length()));
        double next_edge_length = CGAL::sqrt(CGAL::to_double(next_edge.squared_length()));
        CGAL::Vector_2<Kernel> bisector = next_edge_length * prev_edge + prev_edge_length * next_edge;
        double bisector_length = CGAL::sqrt(CGAL::to_double(bisector.squared_length()));
        bisector = bisector / bisector_length;

        Point2 inset_vertex = *vertex + inset_distance * bisector.perpendicular(CGAL::COUNTERCLOCKWISE);
        inset_polygon.push_back(inset_vertex);
    }
    return inset_polygon;
}

} // namespace cpp_occlusions