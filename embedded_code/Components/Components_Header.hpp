#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "../IncludesHeader.hpp"
#include "../MathHelpers/MathHelpers.hpp"
#include "../Drivers/All_drivers_Header.hpp"

extern Bno085Wraper imu;
extern VL53L1XSensorWraper sensor;

class DroneChassis
{
public:
    DroneChassis();
    //~DroneChassis();

    // Get latest orientation; respects frequency limit
    void setYaw(double rads);
    void setRoll(double rads);
    void setPitch(double rads);
    void setHeight(double DistMM);

    void drive(double outHeight, double outYaw, double outPitch, double outRoll);
    void update();

    PIDCoefficients HeightCoefs;
    PIDCoefficients YawCoefs;
    PIDCoefficients PitchCoefs;
    PIDCoefficients RollCoefs;

//private:
    double toRadians(double degrees);
    double getAngleDifference(double target, double current);
    double normalizeToMinusPiToPi(double angle);
    double TargetYaw = 0;
    double TargetPitch = 0;
    double TargetRoll = 0;
    double TargetHeight = 0;

    PIDController HeightPID;
    PIDController YawPID;
    PIDController PitchPID;
    PIDController RollPID;

    Motor fr,fl,br,bl;
};


#endif