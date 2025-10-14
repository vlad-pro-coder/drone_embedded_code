import time
import math
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from scipy.spatial.transform import Rotation as R

def quaternion_to_euler(w, x, y, z):
    """Convert quaternion (w, x, y, z) to Euler angles in degrees."""
    # Roll (x-axis)
    sinr_cosp = 2.0 * (w*x + y*z)
    cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis)
    sinp = 2.0 * (w*y - z*x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis)
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)

def slerp(q1, q2, t):
    """Spherical linear interpolation between two quaternions."""
    dot = sum(a*b for a,b in zip(q1, q2))
    if dot < 0.0:
        q2 = [-x for x in q2]
        dot = -dot
    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # Very close: linear interp
        result = [a + t*(b-a) for a,b in zip(q1,q2)]
        norm = math.sqrt(sum(x*x for x in result))
        return [x/norm for x in result]
    theta_0 = math.acos(dot)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot*sin_theta/sin_theta_0
    s1 = sin_theta/sin_theta_0
    return [s0*a + s1*b for a,b in zip(q1,q2)]


class SmoothedBNO08x:
    def __init__(
        self,
        i2c,
        address=0x4A,
        use_game_vector=True,
        interval_us=20000,
        alpha=0.9,
        spike_threshold=0.2,
    ):
        """
        alpha: smoothing factor (0 < alpha < 1)
        spike_threshold: maximum allowed difference per quaternion component
        """
        self.use_game_vector = use_game_vector
        self.bno = BNO08X_I2C(i2c, address=address)
        self.feature = (
            BNO_REPORT_GAME_ROTATION_VECTOR
            if use_game_vector
            else BNO_REPORT_ROTATION_VECTOR
        )
        self.interval_us = interval_us
        self.last_q = None
        self.smoothed_q = None
        self.freeze_count = 0
        self.max_freeze = 10
        self.alpha = alpha
        self.spike_threshold = spike_threshold
        self.enable_feature()

    def enable_feature(self):
        try:
            self.bno.enable_feature(self.feature, interval=self.interval_us)
        except TypeError:
            # fallback for older library versions
            self.bno.enable_feature(self.feature)

    def get_quaternion(self):
        try:
            # Read raw quaternion from sensor
            raw_q = self.bno.game_quaternion if self.use_game_vector else self.bno.quaternion
        except Exception as e:
            print("⚠️ failed quaternion:", e)
            #self._reset()
            return None

        # Validate quaternion
        if not isinstance(raw_q, (tuple, list)) or len(raw_q) != 4:
            return None

        # --- Freeze detection on raw data ---
        i, j, k, w = raw_q
        rawyaw,rawpitch,rawroll = quaternion_to_euler(w, i, j, k)
        print(f"rawYaw: {rawyaw:.2f}°, rawPitch: {rawpitch:.2f}°, rawRoll: {rawroll:.2f}°")
        if tuple(raw_q) == getattr(self, 'last_raw_q', None):
            self.freeze_count += 1
            if self.freeze_count >= self.max_freeze:
                print("⚠️ Frozen quaternion detected — resetting feature")
                self._reset()
                self.freeze_count = 0
                return None
        else:
            self.freeze_count = 0

        # Store raw quaternion for next iteration
        self.last_raw_q = tuple(raw_q)

        # Convert to list for spike rejection and smoothing
        q = list(raw_q)

        # --- Spike rejection ---
        if self.smoothed_q is not None:
            for i in range(4):
                if abs(q[i] - self.smoothed_q[i]) > self.spike_threshold:
                    q[i] = self.smoothed_q[i]

        # --- Low-pass smoothing ---
        if self.smoothed_q is None:
            self.smoothed_q = q
        else:
            self.smoothed_q = slerp(self.smoothed_q, q, 1 - self.alpha)

        # Return smoothed quaternion as a tuple
        return tuple(self.smoothed_q)

    def get_euler(self):
        q = self.get_quaternion()
        if q:
            i, j, k, w = q
            return quaternion_to_euler(w, i, j, k)
        return None

    def _reset(self):
        try:
            self.enable_feature()
        except Exception as e:
            print("⚠️ Reset failed:", e)
# ---- Example usage ----
if __name__ == "__main__":
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    imu = BNO08X_I2C(i2c, address=0x4a)

    imu.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

    while True:
        time.sleep(0.05)
        raw_q = []
        try:
            raw_q = imu.game_quaternion
        except Exception as e:
            print("error packet")
            continue

        x, y, z, w = raw_q
        r = R.from_quat([x, y, z, w])

        euler = r.as_euler('xyz', degrees=True)
        roll, pitch, yaw = euler

        print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")