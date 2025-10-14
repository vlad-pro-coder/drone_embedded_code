import SmoothBno085
import VL53L1XSensor
import board, busio
import time
from PythonRelatedInitialization import PythonRelatedInitializer

#PythonRelatedInitializer.initialize()
#imu = SmoothBno085.SmoothedBNO08x(PythonRelatedInitializer.i2c,RefreshFrequency = 200)

distance_sensor = VL53L1XSensor.VL53L1XSensor()

while True:
#while True:
    try:
        print(f"{distance_sensor.get_distance()}")
    except Exception:
        distance_sensor.close()
    #euler = imu.get_euler()
    #if euler:
    #    yaw, pitch, roll = euler
    #    print(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
