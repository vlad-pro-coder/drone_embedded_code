#include "./All_drivers_Header.hpp"
#include <pybind11/embed.h>
namespace py = pybind11;

struct VL53L1XSensorWraper::sensor_dependencies {
    py::object sensor;
};

VL53L1XSensorWraper::VL53L1XSensorWraper() {
    this->sensor_depend = new sensor_dependencies();  // allocate on heap

    auto sys = py::module::import("sys");
    sys.attr("path").attr("insert")(0, "../pythonTest/env/lib/python3.11/site-packages");
    sys.attr("path").attr("append")("../pythonTest/build/lib.linux-aarch64-cpython-311");

    auto sensorMod = py::module::import("VL53L1XSensor");
    this->sensor_depend->sensor = sensorMod.attr("VL53L1XSensor")();
}

double VL53L1XSensorWraper::getDistance() {
    return this->sensor_depend->sensor.attr("get_distance")().cast<double>();
}

// Optional: destructor to free memory
VL53L1XSensorWraper::~VL53L1XSensorWraper() {
    delete this->sensor_depend;
}