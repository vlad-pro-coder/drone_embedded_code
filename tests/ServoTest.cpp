#include <pigpio.h>
#include <iostream>
#include <unistd.h> // for usleep

int main() {
    if (gpioInitialise() < 0) return 1;

    int pin = 16; // GPIO
    //1600 90 de grade servo jos stabilizator
    //1450 90 grade servo sus stabilizator
    //front este gps
    //pin 22 front left
    //pin 23 front right
    //pin 24 back right
    //pin 25 back left

    //1100-2100

    int pulse;

    while (true) {
        std::cout << "Enter pulse width in microseconds (1000-2000, 0 to quit): ";
        std::cin >> pulse;


        gpioServo(pin, pulse);             // send pulse
        std::cout << "Sent " << pulse << " µs to servo.\n";
    }

    gpioServo(pin, 0); // stop PWM
    gpioTerminate();
    return 0;
}
