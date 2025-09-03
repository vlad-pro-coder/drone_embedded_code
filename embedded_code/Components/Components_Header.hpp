#ifndef COMPONENTS_H
#define COMPONENTS_H

#include "../MathHelpers/MathHelpers.hpp"
#include "../Drivers/All_drivers_Header.hpp"

IMU_BNO085 imu;

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

private:
    PIDController HeightPID;
    PIDController YawPID;
    PIDController PitchPID;
    PIDController RollPID;

    Motor m1,m2,m3,m4;
};


#endif