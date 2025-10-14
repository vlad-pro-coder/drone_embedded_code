#include "./All_drivers_Header.hpp"

Servo::Servo(int pin){
    this->servoPin = pin;
    /*if (gpioInitialise() < 0) {
        std::cerr << "Pigpio initialization failed!" << std::endl;
        return 1;
    }*/
   //gpioTerminate()
    setPosition(90);
}

void Servo::setPosition(double angle){
    if(angle == this->currPos)
        return ;
    this->currPos = min(this->maxAngle,max(this->minAngle,angle));
    int currentPulse = static_cast<int>(this->minPulseWidth + (this->currPos - this->minAngle) * (this->maxPulseWidth - this->minPulseWidth) / (this->maxAngle - this->minAngle));
    gpioServo(this->servoPin,currentPulse);
}

Servo::~Servo(){
    setPosition(90);
}

