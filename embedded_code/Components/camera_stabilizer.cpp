#include "./All_drivers_Header.hpp"
#include <iostream>
#include <cmath>
#include <wiringPi.h>
#include <softPwm.h>
using namespace std;
// configurare temporara
#define SERVO_PITCH_PIN 0 // wiringPi pin 0 = GPIO17
#define SERVO_ROLL_PIN  1 // wiringPi pin 1 = GPIO18

// map angle to PWM
int angleToPWM(int angle) {
    return 50 + (angle * 200 / 180); // 50–250 range for softPwm
}

int main() {
    wiringPiSetup();
    softPwmCreate(SERVO_PITCH_PIN, 0, 250);
    softPwmCreate(SERVO_ROLL_PIN, 0, 250);

    IMU_BNO085 imu(1, 50); // 50Hz update rate

    const int PITCH_CENTER = 90;
    const int ROLL_CENTER  = 90;

    int pitchServo = PITCH_CENTER;
    int rollServo  = ROLL_CENTER;

    while (true) {
        Orientation ori = imu.get_orientation();

        // Safety limit: ±40° tilt
        if (fabs(ori.pitch) > 40.0 || fabs(ori.roll) > 40.0) {
            cout << "Tilt limit exceeded! Pitch: " << ori.pitch
                      << " Roll: " << ori.roll << endl;
            usleep(20000);
            continue;
        }

        // Opposite servo movement for stabilization
        pitchServo = PITCH_CENTER - ori.pitch;
        rollServo  = ROLL_CENTER - ori.roll;

        // Constrain servo range
        pitchServo = max(0, min(180, pitchServo));
        rollServo  = max(0, min(180, rollServo));

        // Send to servos
        softPwmWrite(SERVO_PITCH_PIN, angleToPWM(pitchServo));
        softPwmWrite(SERVO_ROLL_PIN, angleToPWM(rollServo));

        // Debug output
        cout << "Pitch: " << ori.pitch << " Roll: " << ori.roll
                  << " | ServoPitch: " << pitchServo
                  << " ServoRoll: " << rollServo << endl;

        usleep(20000); // ~50 Hz update
    }

    return 0;
}
