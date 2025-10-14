#include <pigpio.h>
#include <iostream>
#include <unistd.h> // for usleep

//g++ ServoTest.cpp -o servo -lpigpio -lpthread

int main() {
    if (gpioInitialise() < 0) return 1;

    int pin = 22; // GPIO
    //1600 90 de grade servo jos stabilizator
    //1450 90 grade servo sus stabilizator
    //front este invers gps
    //pin 22 back left
    //pin 23 front right
    //pin 24 back right
    //pin 25 front left
    //

    //750-2300

    int pulse;
    //std::cout << "enter pin"<<'\n';
    //std::cin >> pin;

    while (true) {
        std::cout << "Enter pulse width in microseconds (1000-2000, 0 to quit): ";
        std::cin >> pulse;


        gpioServo(pin, pulse);             // send pulse
        //gpioServo(23, pulse);
        //gpioServo(24, pulse);
        //gpioServo(25, pulse);
        std::cout << "Sent " << pulse << " µs to servo.\n";
    }

    gpioServo(pin, 0); // stop PWM
    gpioTerminate();
    return 0;
}
