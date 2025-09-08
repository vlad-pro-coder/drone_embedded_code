#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <vector>
#include <cmath>
#include <tuple>

#define I2C_DEVICE "/dev/i2c-1"
#define BNO085_ADDR 0x4A

// Convert 16-bit little-endian to float (Q14 -> 1/16384)
float s16_to_float(int16_t val) {
    return val / 16384.0f;
}

// Quaternion to Euler angles
std::tuple<double,double,double> quatToEuler(double w,double x,double y,double z) {
    double ysqr = y*y;

    double t0 = +2.0 * (w*x + y*z);
    double t1 = +1.0 - 2.0*(x*x + ysqr);
    double roll = atan2(t0,t1);

    double t2 = +2.0*(w*y - z*x);
    t2 = (t2 > 1.0) ? 1.0 : ((t2 < -1.0) ? -1.0 : t2);
    double pitch = asin(t2);

    double t3 = +2.0*(w*z + x*y);
    double t4 = +1.0 - 2.0*(ysqr + z*z);
    double yaw = atan2(t3,t4);

    return {yaw,pitch,roll};
}

int main() {
    int fd = open(I2C_DEVICE, O_RDWR);
    if(fd < 0) { perror("open"); return 1; }

    if(ioctl(fd, I2C_SLAVE, BNO085_ADDR) < 0) {
        perror("ioctl");
        return 1;
    }

    // Set Feature Command: Rotation Vector (0x05)
    uint8_t enable_cmd[21] = {
    0x15, 0x00,       // Packet length = 21
    0x02, 0x00,       // Channel 2, sequence 0
    0xFD, 0x28,       // Set Feature + Feature ID (Rotation Vector)
    0x00, 0x00, 0x00, 0x00, // Feature flags + Change sensitivity (unused)
    0xC8, 0x00, 0x00, 0x00, // Report interval = 200 ms (0x000000C8)
    0x00, 0x00, 0x00, 0x00, // Batch interval
    0x00, 0x00, 0x00        // Reserved
};
    if(write(fd, enable_cmd, sizeof(enable_cmd)) != sizeof(enable_cmd)) {
        perror("write Set Feature");
    }

    std::cout << "Waiting for rotation vector data...\n";

    while(true) {
        uint8_t header[4] = {0};
        if(read(fd, header, 4) != 4) {
            usleep(5000);
            continue;
        }

        int packet_len = header[0] | (header[1] << 8);
        if(packet_len <= 4 || packet_len > 512) continue;

        std::vector<uint8_t> payload(packet_len - 4, 0);
        if(read(fd, payload.data(), packet_len - 4) != packet_len - 4) continue;

        // Print raw header
        std::cout << "Header: ";
        for(int i=0;i<4;i++)
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)header[i] << " ";
        std::cout << "\n";

        // Print raw payload
        std::cout << "Payload: ";
        for(auto b : payload)
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)b << " ";
        std::cout << "\n";

        // --- Parse rotation vector quaternion (Report ID 0x05) ---
        if(payload.size() >= 14 && payload[0] == 0x05) {
            int16_t q[4];
            q[0] = payload[4] | (payload[5]<<8);   // i
            q[1] = payload[6] | (payload[7]<<8);   // j
            q[2] = payload[8] | (payload[9]<<8);   // k
            q[3] = payload[10] | (payload[11]<<8); // real

            float qw = s16_to_float(q[3]);
            float qx = s16_to_float(q[0]);
            float qy = s16_to_float(q[1]);
            float qz = s16_to_float(q[2]);

            std::cout << std::fixed << std::setprecision(4);
            std::cout << "Quaternion: [" << qw << "," << qx << "," << qy << "," << qz << "]\n";

            auto [yaw,pitch,roll] = quatToEuler(qw,qx,qy,qz);
            std::cout << "Yaw: " << yaw*180/M_PI
                      << " Pitch: " << pitch*180/M_PI
                      << " Roll: " << roll*180/M_PI << "\n\n";
        }

        usleep(20000); // ~50Hz
    }

    close(fd);
    return 0;
}
