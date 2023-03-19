#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../include/cpp_occlusions/occlusion_handler.h"

#include <iostream>

namespace py = pybind11;

PYBIND11_MODULE(py_occlusions, m) {
    // Optional docstring
    m.doc() = "Library to find bounds on states of hidden traffic and predict their future occupancies";
    
    py::class_<cpp_occlusions::OcclusionHandler>(m, "OcclusionHandler")
        .def(py::init<PolygonListBinding, PolygonBinding, int, cpp_occlusions::ReachabilityParams>())
        .def("update", &cpp_occlusions::OcclusionHandler::Update, py::return_value_policy::copy)
        .def("get_reachable_sets", &cpp_occlusions::OcclusionHandler::GetReachableSets, py::return_value_policy::copy);

    py::class_<cpp_occlusions::ReachabilityParams>(m, "ReachabilityParams")
        .def(py::init<>())
        .def_readwrite("vmin", &cpp_occlusions::ReachabilityParams::vmin)
        .def_readwrite("vmax", &cpp_occlusions::ReachabilityParams::vmax)
        .def_readwrite("amin", &cpp_occlusions::ReachabilityParams::amin)
        .def_readwrite("amax", &cpp_occlusions::ReachabilityParams::amax)
        .def_readwrite("dt", &cpp_occlusions::ReachabilityParams::dt)
        .def_readwrite("prediction_horizon", &cpp_occlusions::ReachabilityParams::prediction_horizon)
        .def_readwrite("min_shadow_area", &cpp_occlusions::ReachabilityParams::min_shadow_area);

}

namespace PYBIND11_NAMESPACE { namespace detail {
    template <> struct type_caster<Polygon> {

        Polygon ShapelyToCGAL(PyObject *shapely_polygon) {
            Polygon cgal_polygon;
            
            Py_ssize_t i, n;
            Py_ssize_t x = 0;
            Py_ssize_t y = 1;

            PyObject* xy = PyObject_GetAttrString(PyObject_GetAttrString(PyObject_GetAttrString(shapely_polygon, "exterior"), "coords"), "xy");
            
            PyObject* x_coords = PyTuple_GetItem(xy, x);
            PyObject* y_coords = PyTuple_GetItem(xy, y);

            PyObject* index;
            PyObject* xitem;
            PyObject* yitem;

            n = PyObject_Length(x_coords);

            for(i = 0; i < n-1; ++i) {
                index = PyLong_FromLong(i);
                xitem = PyObject_GetItem(x_coords, index);
                yitem = PyObject_GetItem(y_coords, index);
                
                cgal_polygon.push_back(Point2(PyFloat_AsDouble(xitem),PyFloat_AsDouble(yitem)));
            }

            return cgal_polygon;
        }

        static py::object CGALToShapely(Polygon cgal_polygon) {
            py::object ShapelyPolygon = py::module_::import("shapely").attr("geometry").attr("Polygon");

            Py_ssize_t i, n;
            Py_ssize_t x = 0;
            Py_ssize_t y = 1;

            n = cgal_polygon.size();

            PyObject* coords = PyList_New(n+1);
            PyObject* coord = PyList_New(2);

            PyObject* xitem;
            PyObject* yitem;

            for(i = 0; i < n; i++) {
                xitem = PyFloat_FromDouble((double) cgal_polygon.vertex(i).x());
                yitem = PyFloat_FromDouble((double) cgal_polygon.vertex(i).y());
                PyList_SetItem(coord, x, xitem);
                PyList_SetItem(coord, y, yitem);
                PyList_SetItem(coords, i, coord);
            }

            // ShapelyPolygons have the first point added as last point to close the loop
            xitem = PyFloat_FromDouble((double) handle(shapely)cgal_polygon.vertex(0).x());
            yitem = PyFloat_FromDouble((double) cgal_polygon.vertex(0).y());
            PyList_SetItem(coord, x, xitem);
            PyList_SetItem(coord, y, yitem);
            PyList_SetItem(coords, n, coord);

            py::object xy_coords = reinterpret_steal<py::object>(coords);
            std::cout << "Retrieved xy_coords: " << xy_coords << std::endl;

            py::object shapely = ShapelyPolygon(xy_coords);
            std::cout << "CGALToShapely function complete, converted object: " << shapely << "\n";
            return shapely;
        }

    public:
    
        PYBIND11_TYPE_CASTER(Polygon, const_name("CGALPolygon"));

        // Conversion part 1 (Python->C++)
        bool load(handle src, bool) {
            PyObject *source = src.ptr();
            value = ShapelyToCGAL(source);
            return !PyErr_Occurred();
        }

        // Conversion part 2 (C++ -> Python)
        static handle cast(Polygon src, return_value_policy, handle /* parent */) {
            std::cout << "We are in the cast method with source" << src << "\n";
            py::object shapely = CGALToShapely(src);
            std::cout << "cast function complete, our casted object is: " << shapely << "\n";
            py::handle shapely_handle = shapely.release();
            return shapely_handle;
        }
    };
}}