#include <iostream>
#include <pybind11/embed.h>

namespace py = pybind11;
using namespace std;

int main() {
    py::scoped_interpreter guard{};  // start Python interpreter

    try {
        // Add Python module path
        auto sys = py::module::import("sys");
        sys.attr("path").attr("insert")(0, "../pythonTest/env/lib/python3.11/site-packages");
        sys.attr("path").attr("append")("../pythonTest/build/lib.linux-aarch64-cpython-311");

        // Import Python modules
        auto pyInitMod = py::module::import("PythonRelatedInitialization");
        auto sensorMod = py::module::import("VL53L1XSensor");

        // Initialize I2C
        //auto pyInitializer = pyInitMod.attr("PythonRelatedInitializer");
        //pyInitializer.attr("initialize")();  // static method
        //auto i2c_bus = pyInitializer.attr("i2c");

        // Access the sensor class
        auto sensor = sensorMod.attr("VL53L1XSensor")();

        // Read distance 100 times
        while(true) {
            double dist = sensor.attr("get_distance")().cast<double>();
            cout << dist << " mm\n";
        }


        // Close sensor
        sensor.attr("close")();

    } catch (const py::error_already_set &e) {
        cerr << "Python error: " << e.what() << endl;
    }

    return 0;
}
