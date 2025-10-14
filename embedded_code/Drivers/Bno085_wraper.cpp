#include "./All_drivers_Header.hpp"
#include <pybind11/embed.h>
namespace py = pybind11;

struct Bno085Wraper::imu_dependencies {
    py::object imu;
    py::object i2c_bus;
};

Bno085Wraper::Bno085Wraper() {
    this->imu_depend = new imu_dependencies();  // allocate on heap

    auto sys = py::module::import("sys");
    sys.attr("path").attr("insert")(0, "../pythonTest/env/lib/python3.11/site-packages");
    sys.attr("path").attr("append")("../pythonTest/build/lib.linux-aarch64-cpython-311");

    auto pyInitMod = py::module::import("PythonRelatedInitialization");
    auto imuMod = py::module::import("SmoothBno085");

    auto pyInitializer = pyInitMod.attr("PythonRelatedInitializer");
    pyInitializer.attr("initialize")();

    this->imu_depend->i2c_bus = pyInitializer.attr("i2c");
    this->imu_depend->imu = imuMod.attr("SmoothedBNO08x")(
        this->imu_depend->i2c_bus,
        py::arg("RefreshFrequency") = 1000
    );
}

tuple<double,double,double> Bno085Wraper::getAngles() {
    auto euler = this->imu_depend->imu.attr("get_euler")();
    py::tuple euler_tuple = euler.cast<py::tuple>();
    return make_tuple(
        euler_tuple[0].cast<double>(),
        euler_tuple[1].cast<double>(),
        euler_tuple[2].cast<double>()
    );
}

// Optional: destructor to free memory
Bno085Wraper::~Bno085Wraper() {
    delete this->imu_depend;
}