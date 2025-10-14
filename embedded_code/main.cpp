#include "./IncludesHeader.hpp"
#include "./Drivers/All_drivers_Header.hpp"
#include "./MathHelpers/MathHelpers.hpp"
#include "./Components/Components_Header.hpp"
#include <pybind11/embed.h> 
namespace py = pybind11;

atomic<char> pidc{'x'};
atomic<double> coefp{0}, coefi{0}, coefd{0};
atomic<bool> started{false};
atomic<bool> running{true};

void inputThread() {
    while(running.load()) {
        char code;
        double a,b,c;
        if(cin >> code) {
            if(code == 's') {
                started.store(true);  // start signal
                cout << "Starting main loop...\n";
            }
            else if(code == 'c') {
                running.store(false); // stop everything
            }
            else if(cin >> a >> b >> c) { // PID update
                pidc.store(code);
                coefp.store(a);
                coefi.store(b);
                coefd.store(c);
            }
        }
    }
}

py::scoped_interpreter guard{};  // start Python interpreter

Bno085Wraper imu;
VL53L1XSensorWraper sensor;

int main() {
    
    thread t_input(inputThread);
    DriversInitializer::initialize();
    DroneChassis drone;

    while(!started.load() && running.load()) {
        this_thread::sleep_for(10ms);
    }

    try {

        while(running.load()){
            char code = pidc.load();
            if(code == 'y')
                drone.YawPID.setPidCoefficients(PIDCoefficients(coefp.load(),coefi.load(),coefd.load()));
            else if(code == 'p')
                drone.PitchPID.setPidCoefficients(PIDCoefficients(coefp.load(),coefi.load(),coefd.load()));
            else if(code == 'r')
                drone.RollPID.setPidCoefficients(PIDCoefficients(coefp.load(),coefi.load(),coefd.load()));
            else if(code == 'c')
                break;
            drone.update();
        }

        t_input.join();

    } catch (const py::error_already_set &e) {
        cerr << "Python error: " << e.what() << endl;
    }

    DriversInitializer::CleanUp();

    return 0;
}
