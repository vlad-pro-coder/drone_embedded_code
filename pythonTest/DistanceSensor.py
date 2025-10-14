import qwiic_vl53l1x
import time

# Create sensor object
sensor = qwiic_vl53l1x.QwiicVL53L1X()

# Attempt to initialize the sensor
try:
    sensor.sensor_init()
    print("Sensor initialized successfully.")
except Exception as e:
    print(f"Failed to initialize sensor: {e}")
    exit(1)

# Start ranging
try:
    sensor.start_ranging()
    print("Ranging started.")
except Exception as e:
    print(f"Failed to start ranging: {e}")
    sensor.stop_ranging()
    exit(1)

# Read distance measurements
try:
    for _ in range(10):
        distance = sensor.get_distance()
        print(f"Distance: {distance} mm")
        time.sleep(0.1)
except Exception as e:
    print(f"Error during measurement: {e}")
finally:
    sensor.stop_ranging()
    print("Ranging stopped.")
