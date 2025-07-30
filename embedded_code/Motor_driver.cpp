#include "./All_drivers_Header.hpp"

Motor::Motor(int pin){
    this->motorPin = pin
    /*if (gpioInitialise() < 0) {
        std::cerr << "Pigpio initialization failed!" << std::endl;
        return 1;
    }*/
   //gpioTerminate()
    setPower(0);
}

void Motor::setPower(double power){
    if(power == this->currPower)
        return ;
    this->currPower = min(this->maxPower,max(this->minPower,power));
    this->currentPulse = static_cast<int>(this->currentUsedPulseMin + (currPower - this->minPower) * (this->currentUsedPulseMax - this->currentUsedPulseMin) / (this->maxPower - this->minPower));
}
void Motor::DisableMotor(){
    this->isDisabled = true;
}
void Motor::EnableMotor(){
    this->isDisabled = false;
}
void Motor::EnableOneShot(){
    this->currentUsedPulseMin = this->minPulseWidthOneShot
    this->currentUsedPulseMax = this->maxPulseWidthOneShot
}
void Motor::DisableOneShot(){
    this->currentUsedPulseMin = this->minPulseWidthPWM
    this->currentUsedPulseMax = this->maxPulseWidthPWM
}

void Motor::update(){
    if(this->isDisabled)
        gpioSetServoPulsewidth(this->motorPin, 0);
    else
        gpioSetServoPulsewidth(this->motorPin, this->currentPulse);
}

Motor::~Motor(){
    gpioSetServoPulsewidth(this->motorPin, 0);  // disable PWM
}