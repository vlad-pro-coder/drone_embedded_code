#include "./All_drivers_Header.hpp"

Motor::Motor(int pin){
    this->motorPin = pin;
    setPower(0);
}

void Motor::setPower(double power){
    power = min(this->maxPower,max(this->minPower,power));
    if(power == this->currPower || this->isDisabled)
        return ;
    this->currPower = power;
    int currentPulse = static_cast<int>(this->currentUsedPulseMin + (this->currPower - this->minPower) * (this->currentUsedPulseMax - this->currentUsedPulseMin) / (this->maxPower - this->minPower));
    gpioServo(this->motorPin,currentPulse);
}
void Motor::DisableMotor(){
    this->isDisabled = true;
}
void Motor::EnableMotor(){
    this->isDisabled = false;
}

Motor::~Motor(){
    setPower(0);
}