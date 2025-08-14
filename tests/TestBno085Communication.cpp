#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define BNO085_ADDR 0x4A // Change to 0x4B if SA0 high

int main() {
    const char *i2cDevice = "/dev/i2c-1";
    int file;

    // Open I2C device
    if ((file = open(i2cDevice, O_RDWR)) < 0) {
        std::cerr << "Failed to open I2C device\n";
        return 1;
    }

    // Set I2C slave address
    if (ioctl(file, I2C_SLAVE, BNO085_ADDR) < 0) {
        std::cerr << "Failed to set I2C address\n";
        close(file);
        return 1;
    }

    // Try reading first few bytes (SHTP header)
    uint8_t buffer[4] = {0};
    if (read(file, buffer, sizeof(buffer)) != sizeof(buffer)) {
        std::cerr << "Failed to read from BNO085\n";
        close(file);
        return 1;
    }

    std::cout << "Read " << sizeof(buffer) << " bytes from BNO085: ";
    for (int i = 0; i < 4; i++) {
        std::cout << "0x" << std::hex << (int)buffer[i] << " ";
    }
    std::cout << std::dec << "\n";

    close(file);
    return 0;
}
