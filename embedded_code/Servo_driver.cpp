#include "./All_drivers_Header.hpp"

Servo::Servo(int pin){
    this->servoPin = pin;
    /*if (gpioInitialise() < 0) {
        std::cerr << "Pigpio initialization failed!" << std::endl;
        return 1;
    }*/
   //gpioTerminate()
    setPosition(90)
}

void Servo::setPosition(double angle){
    if(angle == currAngle)
        return ;
    this->currAngle = min(this->maxAngle,max(this->minAngle,angle));
    this->currentPulse = static_cast<int>(this->minPulseWidth + (currAngle - this->minAngle) * (this->maxPulseWidth - this->minPulseWidth) / (this->maxAngle - this->minAngle));
}

void Servo::DisableServo(){
    this->IsDisabled = true;
}
void Servo::EnableServo(){
    this->IsDisabled = false;
}

void Servo::update(){
    if(this->isDisabled)
        gpioSetServoPulsewidth(this->servoPin, 0);
    else
        gpioSetServoPulsewidth(this->servoPin, this->currentPulse);
}

Servo::~Servo(){
    gpioSetServoPulsewidth(this->servoPin, 0);
}

