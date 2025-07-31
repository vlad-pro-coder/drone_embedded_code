#include <pigpio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <cstdio>
#include <syslog.h>
#include <csignal>
#include <atomic>
#include <fstream>

class VL53L0XDriver {
private:
    int address;
    int fd;
    uint8_t buffer[2];
    uint8_t regData;
    static std::atomic<bool> running;
    
public:
    VL53L0XDriver(int addr = 0x29) : address(addr), fd(-1) {
        openlog("VL53L0X", LOG_PID, LOG_DAEMON);
        if (gpioInitialise() < 0) {
            syslog(LOG_ERR, "Failed to initialize pigpio");
            exit(1);
        }
        fd = open("/dev/i2c-1", O_RDWR);
        if (fd < 0) {
            syslog(LOG_ERR, "Cannot open I2C device");
            gpioTerminate();
            exit(1);
        }
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            syslog(LOG_ERR, "Cannot set I2C address");
            close(fd);
            gpioTerminate();
            exit(1);
        }
        if (!initializeSensor()) {
            syslog(LOG_ERR, "VL53L0X initialization failed");
            close(fd);
            gpioTerminate();
            exit(1);
        }
        syslog(LOG_INFO, "VL53L0X sensor initialized successfully");
    }
    
    uint16_t readDistance() {
        if (!writeReg(0x00, 0x01)) return 0;
        int timeout = 1000;
        while (timeout-- > 0) {
            if (readReg(0x13) & 0x07) break;
            usleep(1000);
        }
        if (timeout <= 0) return 0;
        uint16_t hi = readReg(0x1E);
        uint16_t lo = readReg(0x1F);
        uint16_t distance = (hi << 8) | lo;
        writeReg(0x0B, 0x01);
        return (distance > 8000) ? 0 : distance;
    }
    
    void logDistance(uint16_t distance) {
        std::ofstream distFile("/tmp/distance_sensor.dat", std::ios::trunc);
        if (distFile.is_open()) {
            distFile << distance << std::endl;
            distFile.close();
        }
    }
    
    static void signalHandler(int sig) {
        running = false;
        syslog(LOG_INFO, "Shutdown signal received");
    }
    
    ~VL53L0XDriver() {
        if (fd >= 0) close(fd);
        gpioTerminate();
        closelog();
    }

private:
    bool initializeSensor() {
        usleep(30000);
        if (readReg(0xC0) != 0xEE) return false;
        if (!writeReg(0x88, 0x00)) return false;
        if (!writeReg(0x80, 0x01)) return false;
        if (!writeReg(0xFF, 0x01)) return false;
        if (!writeReg(0x00, 0x00)) return false;
        if (!writeReg(0x91, 0x3C)) return false;
        if (!writeReg(0x00, 0x01)) return false;
        if (!writeReg(0xFF, 0x00)) return false;
        if (!writeReg(0x80, 0x00)) return false;
        if (!writeReg(0x60, 0x00)) return false;
        usleep(2000);
        return true;
    }
    
    bool writeReg(uint8_t reg, uint8_t value) {
        buffer[0] = reg;
        buffer[1] = value;
        return (write(fd, buffer, 2) == 2);
    }
    
    uint8_t readReg(uint8_t reg) {
        if (write(fd, &reg, 1) != 1) return 0;
        if (read(fd, &regData, 1) != 1) return 0;
        return regData;
    }
};

std::atomic<bool> VL53L0XDriver::running(true);

int main() {
    if (daemon(0, 0) == -1) {
        return 1;
    }
    signal(SIGTERM, VL53L0XDriver::signalHandler);
    signal(SIGINT, VL53L0XDriver::signalHandler);
    try {
        VL53L0XDriver sensor;
        while (VL53L0XDriver::running) {
            uint16_t distance = sensor.readDistance();
            if (distance > 0) {
                sensor.logDistance(distance);
                if (distance < 200) {
                    syslog(LOG_WARNING, "Critical distance: %d mm", distance);
                }
            }
            usleep(50000);
        }
    } catch (...) {
        syslog(LOG_ERR, "Unexpected error occurred");
        return 1;
    }
    return 0;
}
