#ifndef MATHHELPERS_H
#define MATHHELPERS_H

#include "../IncludesHeader.hpp"

template<typename T>
constexpr int signum(T value) {
    return (T(0) < value) - (value < T(0));
}

struct PIDCoefficients{
    double p,i,d;
    PIDCoefficients(double p,double i,double d){
        this->p = p;
        this->i = i;
        this->d = d;
    }
};

class Timer {
private:
    std::chrono::steady_clock::time_point start_time;
public:
    Timer();
    void start();
    void reset();
    double seconds() const;
    double milliseconds() const;
};

class PIDController{
public:
    
    PIDController(PIDCoefficients pidcoef);
    PIDController(double p, double i, double d);
    void setPidCoefficients(PIDCoefficients coeff);
    void setFreq(double f);
    double calculatePower(double currentPosition);
    double calculatePower(double currentPosition, double d);
    void setTargetPosition(double pos, bool resetIsum);
    void setTargetPosition(double pos);
    void setMaxActuatorOutput(double mao);
    double getTargetPosition();
    PIDCoefficients getCoeff();

private:
    unique_ptr<Timer> et;
    unique_ptr<Timer> time;
    PIDCoefficients pidCoefficients;
    double kS = 0;
    double targetPosition = 0;
    double error, lastError, maxActuatorOutput, Isum = 0;
    double lastReturn = 0;
    int clamp = 1;
    double freq = 20;
};





#endif