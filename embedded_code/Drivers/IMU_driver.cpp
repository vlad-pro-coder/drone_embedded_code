#include "./All_drivers_Header.hpp"

IMU_BNO085::IMU_BNO085(int i2cBus, int frequency)
{
    i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0 || ioctl(i2c_fd, I2C_SLAVE, BNO085_ADDR) < 0)
        throw std::runtime_error("Failed to open I2C device");

    initialize();
    setFreq(frequency);
}

IMU_BNO085::~IMU_BNO085()
{
    if (i2c_fd >= 0)
        close(i2c_fd);
}

void IMU_BNO085::initialize()
{
    enable_rotation_vector();
    usleep(50000); // Let sensor settle
}

void IMU_BNO085::enable_rotation_vector()
{
    uint8_t packet[] = {
        0x07, 0x00, // packet length
        0xFD,       // Set Feature Command
        FEATURE_ROTATION_VECTOR,
        0x00, 0x00,
        0x20, 0x4E, 0x00, 0x00, // Interval (50Hz)
        0x00, 0x00};
    write(i2c_fd, packet, sizeof(packet));
}

void IMU_BNO085::read_packet()
{
    read(i2c_fd, buffer, sizeof(buffer));
}

void IMU_BNO085::parse_orientation()
{
    for (int i = 0; i < sizeof(buffer) - 1; i++)
    {
        if (buffer[i] == SENSOR_REPORTID_ROTATION_VECTOR)
        {
            int16_t i_quatReal = buffer[i + 1] | (buffer[i + 2] << 8);
            int16_t i_quatI = buffer[i + 3] | (buffer[i + 4] << 8);
            int16_t i_quatJ = buffer[i + 5] | (buffer[i + 6] << 8);
            int16_t i_quatK = buffer[i + 7] | (buffer[i + 8] << 8);

            float scale = 1.0f / (1 << 14);
            float r = i_quatReal * scale;
            float i_ = i_quatI * scale;
            float j = i_quatJ * scale;
            float k = i_quatK * scale;

            // Convert quaternion to Euler angles
            this->yaw = atan2(2.0 * (i_ * j + r * k), r * r + i_ * i_ - j * j - k * k) * (180.0 / M_PI);
            this->pitch = asin(2.0 * (r * j - i_ * k)) * (180.0 / M_PI);
            this->roll = atan2(2.0 * (r * i_ + j * k), r * r - i_ * i_ - j * j + k * k) * (180.0 / M_PI);
            return;
        }
    }
}

Orientation IMU_BNO085::get_orientation()
{
    read_packet();
    parse_orientation();
    return {this->yaw, this->pitch, this->roll};
}
