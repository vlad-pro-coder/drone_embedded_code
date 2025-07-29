#ifndef ALL_DRIVERS_H
#define ALL_DRIVERS_H

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <pigpio.h>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <iostream>

#define BNO085_ADDR 0x4A
#define CHANNEL_COMMAND 0xF9
#define CHANNEL_EXECUTE 0xFD
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define FEATURE_ROTATION_VECTOR 0x05

struct Orientation {
    float yaw;
    float pitch;
    float roll;
};

class IMU_BNO085 {
public:
    IMU_BNO085(int i2cBus,int frequency);
    ~IMU_BNO085();

    // Get latest orientation; respects frequency limit
    Orientation get_orientation();

private:
    void parse_orientation();
    void read_packet();
    void enable_rotation_vector();
    void initialize();

    int i2c_fd;

    float yaw,pitch,roll;
    
    // I2C device handle, initialization, etc. omitted for brevity
};

class Servo {
    public:
    Servo(int pin);
    ~Servo();

    // Get latest orientation; respects frequency limit
    void setPosition();
    void DisableServo();

private:
    const int minPulseWidth = 1000;  // 1 ms pulse - 0 deg
    const int maxPulseWidth = 2000;  // 2 ms pulse - 180 deg
    const int minAngle = 0;  // 1 ms pulse - 0 deg
    const int maxAngle = 180;  // 2 ms pulse - 180 deg
    int currPos = 90;
    int servoPin = 0;

}

#endif