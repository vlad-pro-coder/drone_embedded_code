import SmoothBno085
import VL53L1XSensor
import board, busio
import time
import sys
from PythonRelatedInitialization import PythonRelatedInitializer

PythonRelatedInitializer.initialize()
imu = SmoothBno085.SmoothedBNO08x(PythonRelatedInitializer.i2c)

#distance_sensor = VL53L1XSensor.VL53L1XSensor()

while True:
    #try:
    #    print(f"{distance_sensor.get_distance()}")
    #except Exception:
    #    distance_sensor.close()
    vel = imu.getVelocity()
    euler = imu.getAngle()
    if euler and vel:
        yaw, pitch, roll = euler
        yawvel, pitchvel, rollvel = vel
        sys.stdout.write("\033[F\033[F")  # Move cursor up two lines
        sys.stdout.write("\033[K")         # Clear line
        print(f"Yaw vel: {yawvel:.2f}°, Pitch vel: {pitchvel:.2f}°, Roll vel: {rollvel:.2f}°")
        sys.stdout.write("\033[K")         # Clear line
        print(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°", flush=True)

    imu.update()

