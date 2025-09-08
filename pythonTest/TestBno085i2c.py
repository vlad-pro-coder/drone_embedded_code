import time
import board
import busio
import math
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

# I2C setup
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c, address=0x4A)

# Enable rotation vector (provides quaternion)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

def quaternion_to_euler(w, x, y, z):
    """Convert quaternion (w, x, y, z) to Euler angles in degrees."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # clamp
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Convert to degrees
    return (math.degrees(yaw), math.degrees(pitch), math.degrees(roll))

while True:
    time.sleep(0.5)
    try:
    # Get quaternion from sensor
        quat_i, quat_j, quat_k, quat_real = bno.quaternion
        yaw, pitch, roll = quaternion_to_euler(quat_real, quat_i, quat_j, quat_k)
        print("Yaw: {:.2f}°, Pitch: {:.2f}°, Roll: {:.2f}°".format(yaw, pitch, roll))
    except Exception as e:
        print(f"⚠️ Skipping quaternion read due to error: {e}")
        bno._data_buffer = bytearray(len(bno._data_buffer))
