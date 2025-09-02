#include "./All_drivers_Header.hpp"
#include <pigpio.h>
#include <csignal>
#include <syslog.h>
#include <unistd.h>
#include <iostream>

#define SERVO_PIN 18 // Servo pin on Raspberry Pi GPIO

#define CLAW_OPEN_PULSE 1500 // microseconds
#define CLAW_CLOSE_PULSE 1000 // microseconds

#define CLOSE_THRESHOLD 200 // mm

// Step size and delay for smooth movement
#define SERVO_STEP 5      // microseconds per step
#define SERVO_DELAY 10    // ms delay per step

static std::atomic<bool> running(true);

void signalHandler(int sig)
{
    running = false;
    syslog(LOG_INFO, "Shutdown signal received");
}

// Move servo smoothly
void moveServoSmooth(int pin, int startPulse, int endPulse, int step, int delayMs)
{
    if (startPulse < endPulse)
    {
        for (int pulse = startPulse; pulse <= endPulse; pulse += step)
        {
            gpioServo(pin, pulse);
            usleep(delayMs * 1000);
        }
    }
    else
    {
        for (int pulse = startPulse; pulse >= endPulse; pulse -= step)
        {
            gpioServo(pin, pulse);
            usleep(delayMs * 1000);
        }
    }
}

int main()
{
    openlog("ClawController", LOG_PID, LOG_DAEMON);

    if (gpioInitialise() < 0)
    {
        syslog(LOG_ERR, "Failed to initialize pigpio");
        return 1;
    }

    signal(SIGTERM, signalHandler); // kill <pid> command to exit while loop (cleanup stop)
    signal(SIGINT, signalHandler); // Ctrl+C to stop while loop (instant stop)

    try
    {
        VL53L0XDriver sensor;

        gpioSetMode(SERVO_PIN, PI_OUTPUT);
        int currentPulse = CLAW_CLOSE_PULSE;
        gpioServo(SERVO_PIN, currentPulse);

        while (running)
        {
            uint16_t distance = sensor.readDistance();

            if (distance > 0)
            {
                syslog(LOG_INFO, "Distance: %d mm", distance);

                if (distance <= CLOSE_THRESHOLD && currentPulse != CLAW_CLOSE_PULSE)
                {
                    syslog(LOG_INFO, "Object detected — closing claw smoothly");
                    moveServoSmooth(SERVO_PIN, currentPulse, CLAW_CLOSE_PULSE, SERVO_STEP, SERVO_DELAY);
                    currentPulse = CLAW_CLOSE_PULSE;
                }
                else if (distance > CLOSE_THRESHOLD && currentPulse != CLAW_OPEN_PULSE)
                {
                    syslog(LOG_INFO, "No object nearby — opening claw smoothly");
                    moveServoSmooth(SERVO_PIN, currentPulse, CLAW_OPEN_PULSE, SERVO_STEP, SERVO_DELAY);
                    currentPulse = CLAW_OPEN_PULSE;
                }
            }
            usleep(50000);
        }
    }
    catch (...)
    {
        syslog(LOG_ERR, "Unexpected error occurred");
        gpioTerminate();
        return 1;
    }

    gpioServo(SERVO_PIN, 0); 
    gpioTerminate();
    closelog();

    return 0;
}
