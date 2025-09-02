#ifndef COMPONENTS_H
#define COMPONENTS_H


class DroneChassis
{
public:
    DroneChassis(int i2cBus, int frequency);
    ~DroneChassis();

    // Get latest orientation; respects frequency limit
    void setYaw();
    void setRoll();
    void setPitch();

private:
    void parse_orientation();
    void read_packet();
    void enable_rotation_vector();
    void initialize();

    int i2c_fd;

    float yaw, pitch, roll;

    // I2C device handle, initialization, etc. omitted for brevity
};


#endif