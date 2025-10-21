#include "./All_drivers_Header.hpp"

Motor::Motor(int pin,pair<int,int>pwm){
    setPWMinterval(pwm);
    this->motorPin = pin;
    setPower(0);
}

void Motor::setPower(double power){
    power = clamp(power, this->minPower, this->maxPower);
    if(power == this->currPower || this->isDisabled)
        return ;
    this->currPower = power;
    int currentPulse = static_cast<int>(this->currentUsedPulseMin + (this->currPower - this->minPower) * (this->currentUsedPulseMax - this->currentUsedPulseMin) / (this->maxPower - this->minPower));
    cout<<currentPulse<<'\n';
    gpioServo(this->motorPin,currentPulse);
}

void Motor::setPowerSmooth(double power) {
    power = clamp(power, this->minPower, this->maxPower);

    if (power == targetPower || isDisabled)
        return;

    startPower = currPower;
    targetPower = power;

    // Full range (1.0 change) = 1 second
    rampDuration = std::abs(targetPower - startPower) * fullThrottleTime;

    if (rampDuration <= 0.0) {
        currPower = targetPower;
        setPower(targetPower); // use your instant version
        isRamping = false;
        return;
    }

    rampTimer.reset();
    isRamping = true;
}

void Motor::update() {
    if (!isRamping || isDisabled)
        return;

    double t = rampTimer.seconds() / rampDuration;

    if (t >= 1.0) {
        t = 1.0;
        isRamping = false;
    }

    // Linear ramp
    double newPower = startPower + t * (targetPower - startPower);

    // Use your existing setPower() to actually apply it
    setPower(newPower);
}

void Motor::setPWMinterval(pair<int,int> pwm)
{
    this->currentUsedPulseMin = pwm.first;
    this->currentUsedPulseMax = pwm.second;
}

void Motor::DisableMotor(){
    setPower(0);
    this->isDisabled = true;
}
void Motor::EnableMotor(){
    this->isDisabled = false;
}

Motor::~Motor(){
    setPower(0);
}