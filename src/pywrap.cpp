#include <Eigen/Dense>
#include <cmath>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <iostream>

namespace py = pybind11;
constexpr auto byref = py::return_value_policy::reference_internal;

int calc_trajectory(py::array_t<double>& t_waypoints) {
    if (t_waypoints.ndim() != 2)
        throw std::runtime_error("Results should be a 2-D Numpy array");
    auto buf = t_waypoints.request();
    double* ptr = (double*)buf.ptr;
    size_t N = t_waypoints.shape()[0];
    size_t M = t_waypoints.shape()[1];

    int pos = 0;
    for (int i = 0; i < N; i++){
        for (int j = 0; j < M; j ++) {
            std::cout << "Value at (" << i << ", " << j << "): " << ptr[pos] << std::endl;
            ptr[pos] = 1; 
            pos++;
        }
    }
    return 0;
}

PYBIND11_MODULE(MyLib, m) {
    // optional module docstring
    m.doc() = "pybind11 calculator plugin";

    m.def("calc_trajectory", &calc_trajectory, "Calculates a trajectory for given target waypoints and timestamps.");
}