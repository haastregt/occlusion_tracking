from CGAL.CGAL_Polyhedron_3 import Polyhedron_3 as Polyhedron
from CGAL.CGAL_Polyhedron_3 import Polyhedron_modifier, ABSOLUTE_INDEXING
from CGAL.CGAL_Kernel import Point_2, Triangle_2, Point_3
from CGAL import CGAL_Kernel


class PolyHedronPlayground:
    def __init__(self):
        test = Polyhedron()
        test2 = test.make_tetrahedron()
        print(type(test))
        print(type(test2))

        if test.is_tetrahedron(test2):
            print("test is a tetrahedron")
        
        t1=Triangle_2(Point_2(0,0),Point_2(1,0),Point_2(0,1))
        t2=Triangle_2(Point_2(1,1),Point_2(1,0),Point_2(0,1))
        object = CGAL_Kernel.intersection(t1,t2)
        assert object.is_Segment_2()
        print(object.get_Segment_2())

    def build_poly(self):
        m = Polyhedron_modifier()
        P = Polyhedron()

        # First create single triangle
        m.begin_surface(3,1)
        m.add_vertex(Point_3(0, 0, 0))
        m.add_vertex(Point_3(0, 1, 0))
        m.add_vertex(Point_3(1, 0.5, 0))
        m.begin_facet()
        m.add_vertex_to_facet(0)
        m.add_vertex_to_facet(1)
        m.add_vertex_to_facet(2)
        m.end_facet()

        # Append the triangle to P
        P.delegate(m)
        print("(v,f,e) = ", P.size_of_vertices(), P.size_of_facets(), divmod(P.size_of_halfedges(), 2)[0])

        # Clear the incremental builder
        m.clear()

        # Define a new triangle, reusing polyhedron vertices
        m.begin_surface(1,1,0,ABSOLUTE_INDEXING)
        m.add_vertex(Point_3(-1, 0.5, 0))
        m.begin_facet()
        m.add_vertex_to_facet(1)
        m.add_vertex_to_facet(0)
        m.add_vertex_to_facet(3)
        m.end_facet()

        # append new triangle to existing one
        P.delegate(m)
        print("(v,f,e) = ", P.size_of_vertices(), P.size_of_facets(), divmod(P.size_of_halfedges(), 2)[0])

        assert P.is_valid()

    def update(self):
        pass

    def get_currently_occluded_area(self):
        pass

if __name__ == "__main__":
    a = PolyHedronPlayground()
    a.build_poly()
