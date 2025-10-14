#ifndef ALL_DRIVERS_H
#define ALL_DRIVERS_H

#include "../IncludesHeader.hpp"

struct GPS_Data
{
    int message;
    double lat;
    double lon;
};

class DriversInitializer
{
    public:
        static void initialize();
        static void CleanUp();
};

class Servo
{
public:
    Servo(int pin);
    ~Servo();

    // Get latest orientation; respects frequency limit
    void setPosition(double pos);

private:
    const int minPulseWidth = 500; // 1 ms pulse - 0 deg
    const int maxPulseWidth = 2500; // 2 ms pulse - 180 deg
    const double minAngle = 0;      // 1 ms pulse - 0 deg
    const double maxAngle = 180;    // 2 ms pulse - 180 deg
    int currPos = 0;
    int servoPin = 0;
};

class Motor
{
public:
    Motor(int pin);
    ~Motor();

    // Get latest orientation; respects frequency limit
    void setPower(double power);
    void DisableMotor();
    void EnableMotor();

private:
    const int minPulseWidthPWM = 750;
    const int maxPulseWidthPWM = 2300;
    const int minPulseWidthOneShot = 125;
    const int maxPulseWidthOneShot = 250;
    int currentUsedPulseMin = minPulseWidthPWM;
    int currentUsedPulseMax = maxPulseWidthPWM;
    const double minPower = 0.0;
    const double maxPower = 1.0;
    bool isDisabled = false;
    double currPower = -1;
    int motorPin = 0;
};

class Bno085Wraper {
public:
    Bno085Wraper();
    ~Bno085Wraper();
    tuple<double, double, double> getAngles();

private:
    struct imu_dependencies;          // forward declaration
    imu_dependencies* imu_depend;     // raw pointer instead of unique_ptr
};

class VL53L1XSensorWraper {
public:
    VL53L1XSensorWraper();
    ~VL53L1XSensorWraper();
    double getDistance();

private:
    struct sensor_dependencies;          // forward declaration
    sensor_dependencies* sensor_depend;     // raw pointer instead of unique_ptr
};


#endif