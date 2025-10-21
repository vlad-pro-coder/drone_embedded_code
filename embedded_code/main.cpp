#include "./IncludesHeader.hpp"
#include "./Drivers/All_drivers_Header.hpp"
#include "./MathHelpers/MathHelpers.hpp"
#include "./Components/Components_Header.hpp"
#include "./SchedulerControl/SchedulerControl.hpp"
#include <pybind11/embed.h> 
namespace py = pybind11;

atomic<char> pidc{'x'};
atomic<double> coefp{0}, coefi{0}, coefd{0};
atomic<bool> started{false};
atomic<bool> running{true};

atomic<double> power{0.0};

void inputThread() {
    while(running.load()) {
        char code;
        cin>>code;
        if(code == 'p')
            {
                double p;
                cin>>p;
                started.store(true);
                power.store(p);
            }
        else if(code == 'c')
            running.store(false);
        /*double a,b,c;
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
        }*/
    }
}

py::scoped_interpreter guard{};  // start Python interpreter

Bno085Wraper imu;
VL53L1XSensorWraper sensor;

int main() {
    DriversInitializer::initialize();
    thread t_input(inputThread);
    /*DroneChassis drone;

    while(!started.load() && running.load()) {
        this_thread::sleep_for(10ms);
    }*/
    Motor m1(22,make_pair(1080,2000)),m2(23,make_pair(1080,2000)),m3(9,make_pair(750,2300)),m4(25,make_pair(900,2200));

    //22 pulse 1080 la 2000
    //23 pulse 1080 la 2000
    //25 pulse 910 - 2200
    //9 pulse 750 la 2300

    /*Scheduler scheduler;
    scheduler
        .addTask(make_shared<WaitSecondsTask>(2.0))
        .addTask(Task::make(
            [] { cout<<"wait 2.0 secs \n"; },
            [] { return true; }
        ))
        .addTask(make_shared<WaitSecondsTask>(3.0))
        .addTask(Task::make(
            [] { cout<<"wait 3.0 secs \n"; },
            [] { return true; }
        ));*/

    try {

        /*auto sys = py::module::import("sys");
        sys.attr("path").attr("insert")(0, "../pythonTest/env/lib/python3.11/site-packages");
        sys.attr("path").attr("append")("../pythonTest/build/lib.linux-aarch64-cpython-311");

        auto pyClientMod = py::module::import("ClientPhotoSender");
        auto pyClient = pyClientMod.attr("ClientPhotoSender")(
            "192.168.1.10"
        );

        pyClient.attr("start_sending_packets")();*/
        
        /*while(running.load()){
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

        t_input.join();*/

        while(running.load()){
            if(started.load())
            {
                m1.setPowerSmooth(power.load());
                m2.setPowerSmooth(power.load());
                m3.setPowerSmooth(power.load());
                m4.setPowerSmooth(power.load());
                started.store(false);
            }
            m1.update();
            m2.update();
            m3.update();
            m4.update();
        }

        /*while(!scheduler.isSchedulerDone())
        {
            scheduler.update();
        }*/

    } catch (const py::error_already_set &e) {
        cerr << "Python error: " << e.what() << endl;
    }

    DriversInitializer::CleanUp();

    return 0;
}
