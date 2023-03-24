#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "../include/cpp_occlusions/occlusion_handler.h"

namespace py = pybind11;

PYBIND11_MODULE(py_occlusions, m)
{
    // Optional docstring
    m.doc() = "Library to find bounds on states of hidden traffic and predict their future occupancies";

    py::class_<cpp_occlusions::OcclusionHandler>(m, "OcclusionHandler")
        .def(py::init<std::list<cpp_occlusions::Polygon>, cpp_occlusions::Polygon, int,
                      cpp_occlusions::ReachabilityParams>())
        .def("update", &cpp_occlusions::OcclusionHandler::Update)
        .def("get_reachable_sets", &cpp_occlusions::OcclusionHandler::GetReachableSets, py::return_value_policy::copy);

    py::class_<cpp_occlusions::ReachabilityParams>(m, "ReachabilityParams")
        .def(py::init<>())
        .def_readwrite("vmin", &cpp_occlusions::ReachabilityParams::vmin)
        .def_readwrite("vmax", &cpp_occlusions::ReachabilityParams::vmax)
        .def_readwrite("amin", &cpp_occlusions::ReachabilityParams::amin)
        .def_readwrite("amax", &cpp_occlusions::ReachabilityParams::amax)
        .def_readwrite("dt", &cpp_occlusions::ReachabilityParams::dt)
        .def_readwrite("prediction_horizon", &cpp_occlusions::ReachabilityParams::prediction_horizon)
        .def_readwrite("min_shadow_volume", &cpp_occlusions::ReachabilityParams::min_shadow_volume);
}

namespace PYBIND11_NAMESPACE
{
namespace detail
{
template <> struct type_caster<cpp_occlusions::Polygon>
{

    cpp_occlusions::Polygon ShapelyToCGAL(PyObject *shapely_polygon)
    {
        cpp_occlusions::Polygon cgal_polygon;

        Py_ssize_t i, n;
        Py_ssize_t x = 0;
        Py_ssize_t y = 1;

        PyObject *xy = PyObject_GetAttrString(
            PyObject_GetAttrString(PyObject_GetAttrString(shapely_polygon, "exterior"), "coords"), "xy");

        PyObject *x_coords = PyTuple_GetItem(xy, x);
        PyObject *y_coords = PyTuple_GetItem(xy, y);

        PyObject *index;
        PyObject *xitem;
        PyObject *yitem;

        n = PyObject_Length(x_coords);

        for (i = 0; i < n - 1; ++i)
        {
            index = PyLong_FromLong(i);
            xitem = PyObject_GetItem(x_coords, index);
            yitem = PyObject_GetItem(y_coords, index);

            cgal_polygon.push_back(cpp_occlusions::Point2(PyFloat_AsDouble(xitem), PyFloat_AsDouble(yitem)));
        }

        // Avoid memory leaks
        Py_DECREF(xy);
        Py_DECREF(index);
        Py_DECREF(xitem);
        Py_DECREF(yitem);

        return cgal_polygon;
    }

    static py::object CGALToShapely(cpp_occlusions::Polygon cgal_polygon)
    {
        Py_ssize_t i, n;
        Py_ssize_t x = 0;
        Py_ssize_t y = 1;

        n = cgal_polygon.size();

        PyObject *coords = PyList_New(n + 1);

        for (i = 0; i < n; i++)
        {
            // Use this if inexact construction kernel is used
            // PyList_SetItem(coords, i,
            //               PyTuple_Pack(2, PyFloat_FromDouble((double)cgal_polygon.vertex(i).x()),
            //                            PyFloat_FromDouble((double)cgal_polygon.vertex(i).y())));

            // Use this if exact construction kernel is used
            PyList_SetItem(coords, i,
                           PyTuple_Pack(2, PyFloat_FromDouble(CGAL::to_double(cgal_polygon.vertex(i).x())),
                                        PyFloat_FromDouble(CGAL::to_double(cgal_polygon.vertex(i).y()))));
        }
        // ShapelyPolygons have the first point added as last point to close the loop
        // Use this if inexact construction kernel is used
        // PyList_SetItem(coords, n,
        //                PyTuple_Pack(2, PyFloat_FromDouble((double)cgal_polygon.vertex(0).x()),
        //                             PyFloat_FromDouble((double)cgal_polygon.vertex(0).y())));

        // Use this if exact construction kernel is used
        PyList_SetItem(coords, i,
                       PyTuple_Pack(2, PyFloat_FromDouble(CGAL::to_double(cgal_polygon.vertex(0).x())),
                                    PyFloat_FromDouble(CGAL::to_double(cgal_polygon.vertex(0).y()))));

        py::object ShapelyPolygon = py::module_::import("shapely").attr("geometry").attr("Polygon");
        py::object xy_coords = reinterpret_steal<py::object>(coords);
        py::object shapely = ShapelyPolygon(xy_coords);

        return shapely;
    }

  public:
    PYBIND11_TYPE_CASTER(cpp_occlusions::Polygon, const_name("CGALPolygon"));

    // Conversion part 1 (Python->C++)
    bool load(handle src, bool)
    {
        PyObject *source = src.ptr();
        value = ShapelyToCGAL(source);

        return !PyErr_Occurred();
    }

    // Conversion part 2 (C++ -> Python)
    static handle cast(cpp_occlusions::Polygon src, return_value_policy, handle /* parent */)
    {
        py::object shapely = CGALToShapely(src);
        py::handle shapely_handle = shapely.release();

        return shapely_handle;
    }
};
} // namespace detail
} // namespace PYBIND11_NAMESPACE