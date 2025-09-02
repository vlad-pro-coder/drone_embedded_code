#include "./MathHelpers.hpp"

PIDController::PIDController(PIDCoefficients pidcoef)
    : pidCoefficients(pidcoef), // initialize PIDCoefficients directly
      error(0),
      lastError(0),
      maxActuatorOutput(1),
      time(std::make_unique<Timer>()), // initialize smart pointers
      et(std::make_unique<Timer>())
{
}

PIDController::PIDController(double p, double i, double d)
    : pidCoefficients(p, i, d), // initialize PIDCoefficients directly
      error(0),
      lastError(0),
      maxActuatorOutput(1),
      time(std::make_unique<Timer>()), // initialize smart pointers
      et(std::make_unique<Timer>())
{
}

void PIDController::setPidCoefficients(PIDCoefficients coeff)
{
    pidCoefficients = coeff;
}
void PIDController::setFreq(double f) { freq = f; }
double PIDController::calculatePower(double currentPosition)
{
    return calculatePower(currentPosition, NULL);
}
double PIDController::calculatePower(double currentPosition, double d)
{
    if (time->seconds() < 1.0 / freq)
        return lastReturn;
    time.reset();
    error = targetPosition - currentPosition;
    double dtime = et->seconds();

    double P = error;
    double D;
    if (d != NULL)
        D = d;
    else
        D = (error - lastError) / et->seconds();
    Isum += error * dtime;
    double r = pidCoefficients.p * P + pidCoefficients.d * D;
    //        double r = pidCoefficients.p * P + pidCoefficients.d * D;

    if (abs(r) >= maxActuatorOutput && error * r > 0)
    {
        clamp = 0;
        Isum = 0;
    }
    else
        clamp = 1;

    r += pidCoefficients.i * Isum;

    et.reset();

    lastError = error;
    lastReturn = r - kS * signum(error);
    return lastReturn;
}
void PIDController::setTargetPosition(double pos, bool resetIsum)
{
    targetPosition = pos;
    if (resetIsum)
        Isum = 0;
}
void PIDController::setTargetPosition(double pos)
{
    setTargetPosition(pos, true);
}
double PIDController::getTargetPosition()
{
    return targetPosition;
}
void PIDController::setMaxActuatorOutput(double mao)
{
    maxActuatorOutput = mao;
}
PIDCoefficients PIDController::getCoeff()
{
    return pidCoefficients;
}