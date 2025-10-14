#include "./All_drivers_Header.hpp"

Motor::Motor(int pin){
    this->motorPin = pin;
    /*if (gpioInitialise() < 0) {
        std::cerr << "Pigpio initialization failed!" << std::endl;
        return 1;
    }*/
   //gpioTerminate()
    setPower(0);
}

void Motor::setPower(double power){
    if(power == this->currPower || this->isDisabled)
        return ;
    this->currPower = min(this->maxPower,max(this->minPower,power));
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