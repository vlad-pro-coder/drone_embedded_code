#include "./All_drivers_Header.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>

#define SERVO_PIN 18 // Servo pin on Raspberry Pi GPIO
#define CLAW_OPEN_PULSE 1500 // microseconds
#define CLAW_CLOSE_PULSE 1000 // microseconds
#define CLOSE_THRESHOLD 200 // mm
#define SERVO_STEP 5      // microseconds per step
#define SERVO_DELAY 10    // ms delay per step
#define LOG_FILE "/home/pi/trash_coordinates.log"

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

// Log GPS coordinates to file with timestamp and action
void logCoordinates(double lat, double lon, const std::string& action)
{
    std::ofstream logFile(LOG_FILE, std::ios::app);
    if (logFile.is_open())
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        logFile << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S") 
                << " | Action: " << action 
                << " | Latitude: " << std::fixed << std::setprecision(6) << lat 
                << " | Longitude: " << std::fixed << std::setprecision(6) << lon 
                << std::endl;
        
        logFile.close();
        syslog(LOG_INFO, "Logged %s at coordinates: %.6f, %.6f", action.c_str(), lat, lon);
    }
    else
    {
        syslog(LOG_ERR, "Failed to open log file: %s", LOG_FILE);
    }
}

int main()
{
    openlog("TrashCollector", LOG_PID, LOG_DAEMON);
    
    if (gpioInitialise() < 0)
    {
        syslog(LOG_ERR, "Failed to initialize pigpio");
        return 1;
    }
    
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);
    
    try
    {
        // Initialize distance sensor
        VL53L0XDriver sensor;
        
        // Initialize GPS
        GPS gps("/dev/serial0");
        
        // Setup servo for claw control
        gpioSetMode(SERVO_PIN, PI_OUTPUT);
        int currentPulse = CLAW_CLOSE_PULSE;
        gpioServo(SERVO_PIN, currentPulse);
        
        // Track claw state to detect transitions
        bool clawIsOpen = false;
        bool previousClawState = false;
        
        // Create/clear log file header
        std::ofstream logFile(LOG_FILE, std::ios::trunc);
        if (logFile.is_open())
        {
            logFile << "=== Trash Collection GPS Log ===" << std::endl;
            logFile << "Format: Timestamp | Action | Latitude | Longitude" << std::endl;
            logFile << "===================================" << std::endl;
            logFile.close();
        }
        
        syslog(LOG_INFO, "Trash collector drone started successfully");
        
        while (running)
        {
            uint16_t distance = sensor.readDistance();
            
            if (distance > 0)
            {
                // Determine claw state based on distance
                previousClawState = clawIsOpen;
                
                if (distance <= CLOSE_THRESHOLD && currentPulse != CLAW_CLOSE_PULSE)
                {
                    // Object detected - close claw to pick up trash
                    syslog(LOG_INFO, "Object detected at %d mm - closing claw", distance);
                    moveServoSmooth(SERVO_PIN, currentPulse, CLAW_CLOSE_PULSE, SERVO_STEP, SERVO_DELAY);
                    currentPulse = CLAW_CLOSE_PULSE;
                    clawIsOpen = false;
                    
                    // Get GPS coordinates when picking up trash
                    GPS_Data coords = gps.getCoordinates();
                    if (coords.message == 1)
                    {
                        logCoordinates(coords.lat, coords.lon, "PICKUP");
                    }
                    else
                    {
                        syslog(LOG_WARNING, "Failed to get GPS coordinates for pickup");
                    }
                }
                else if (distance > CLOSE_THRESHOLD && currentPulse != CLAW_OPEN_PULSE)
                {
                    // No object nearby - open claw (possibly releasing trash)
                    syslog(LOG_INFO, "No object detected - opening claw");
                    moveServoSmooth(SERVO_PIN, currentPulse, CLAW_OPEN_PULSE, SERVO_STEP, SERVO_DELAY);
                    currentPulse = CLAW_OPEN_PULSE;
                    clawIsOpen = true;
                    
                    // Get GPS coordinates when releasing/opening claw
                    GPS_Data coords = gps.getCoordinates();
                    if (coords.message == 1)
                    {
                        logCoordinates(coords.lat, coords.lon, "RELEASE");
                    }
                    else
                    {
                        syslog(LOG_WARNING, "Failed to get GPS coordinates for release");
                    }
                }
                
                // Log distance periodically for debugging
                if (distance < 500)
                {
                    sensor.logDistance(distance);
                }
            }
            
            usleep(50000); // 50ms delay between readings
        }
        
        // Cleanup on exit
        syslog(LOG_INFO, "Shutting down trash collector drone");
        
        // Final GPS log on shutdown
        GPS_Data finalCoords = gps.getCoordinates();
        if (finalCoords.message == 1)
        {
            logCoordinates(finalCoords.lat, finalCoords.lon, "SHUTDOWN");
        }
        
    }
    catch (const std::exception& e)
    {
        syslog(LOG_ERR, "Exception occurred: %s", e.what());
        gpioTerminate();
        closelog();
        return 1;
    }
    catch (...)
    {
        syslog(LOG_ERR, "Unexpected error occurred");
        gpioTerminate();
        closelog();
        return 1;
    }
    
    // Disable servo and cleanup
    gpioServo(SERVO_PIN, 0);
    gpioTerminate();
    closelog();
    
    return 0;
}