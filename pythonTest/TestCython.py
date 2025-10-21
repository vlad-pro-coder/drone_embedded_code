import SmoothBno085
import VL53L1XSensor
import board, busio
import time
from PythonRelatedInitialization import PythonRelatedInitializer

PythonRelatedInitializer.initialize()
imu = SmoothBno085.SmoothedBNO08x(PythonRelatedInitializer.i2c,RefreshFrequency = 100)

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
        print(f"Yaw vel: {yawvel:.2f}°, Pitch vel: {pitchvel:.2f}°, Roll vel: {rollvel:.2f}°")
        print(f"Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
        time.sleep(0.01)

    imu.update()

