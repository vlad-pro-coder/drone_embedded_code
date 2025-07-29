#include "./All_drivers_Header.hpp"

Servo::Servo(int pin){
    this->servoPin = pin;
    /*if (gpioInitialise() < 0) {
        std::cerr << "Pigpio initialization failed!" << std::endl;
        return 1;
    }*/
   //gpioTerminate()
    gpioServo(servoPin, 1500);
}

void Servo::setPosition(int angle){
    angle = min(this->maxAngle,min(this->minAngle,pos));
    const int Pulse = this->minPulseWidth + (angle - this->minAngle) * (this->maxPulseWidth - this->minPulseWidth) / (this->maxAngle - this->minAngle); 
    gpioServo(this->servoPin, Pulse);
}

void Servo::DisableServo(){
    gpioServo(this->servoPin, 0);
}

Servo::~Servo(){
    gpioServo(this->servoPin, 0);
}