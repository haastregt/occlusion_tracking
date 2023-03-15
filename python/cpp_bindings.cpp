#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cpp_occlusions/occlusion_handler.h>

namespace py = pybind11;

PYBIND11_MODULE(cpp_occlusions, m) {
    // Optional docstring
    m.doc() = "Library to find cpp_occlusions of hidden traffic";
    
    py::class_<cpp_occlusions::OcclusionHandler>(m, "OcclusionHandler")
        .def(py::init<std::vector<Polygon>, Polygon, int, cpp_occlusions::ReachabilityParams>())
        .def("update", &cpp_occlusions::OcclusionHandler::Update)
        .def("get_reachable_sets", &cpp_occlusions::OcclusionHandler::GetReachableSets);

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