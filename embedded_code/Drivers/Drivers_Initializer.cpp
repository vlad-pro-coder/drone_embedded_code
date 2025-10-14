#include "./All_drivers_Header.hpp"

void DriversInitializer::initialize(){
    if (gpioInitialise() < 0)
        cerr << "Pigpio initialization failed!" << endl;
}

void DriversInitializer::CleanUp(){
    gpioTerminate();
}