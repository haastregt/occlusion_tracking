#include <pybind11/pybind11.h>

namespace py = pybind11;

void init_motorcycle(py::module &);

namespace mcl {

PYBIND11_MODULE(reachability, m) {
    // Optional docstring
    m.doc() = "Automobile library";
    
    init_motorcycle(m);
}
}