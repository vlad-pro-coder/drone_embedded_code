# smoothed_bno08x.pyx
# ------------------------------
# Fast Cython version: all quaternions in C arrays (double[4])
# ------------------------------

import math
from cpython cimport bool
cimport cython

# Python sensor interface stays as object
import board
import busio
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C
import time

# ------------------------------
# Helper C functions
# ------------------------------

cdef void quaternion_to_euler(double q[4], double* yaw, double* pitch, double* roll):
    cdef double w = q[3]
    cdef double x = q[0]
    cdef double y = q[1]
    cdef double z = q[2]

    # Pitch
    cdef double sinp = 2.0 * (w*y - z*x)
    if sinp >= 1.0:
        pitch[0] = math.pi/2   # clamp
    elif sinp <= -1.0:
        pitch[0] = -math.pi/2  # clamp
    else:
        pitch[0] = math.asin(sinp)

    # Roll
    cdef double sinr_cosp = 2.0 * (w*x + y*z)
    cdef double cosr_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll[0] = math.atan2(sinr_cosp, cosr_cosp)

    # Yaw
    cdef double siny_cosp = 2.0 * (w*z + x*y)
    cdef double cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw[0] = math.atan2(siny_cosp, cosy_cosp)

cdef class SmoothedBNO08x:
    cdef object bno  # I2C sensor
    cdef double last_euler[3]
    cdef double spike_threshold
    cdef int freeze_count
    cdef int max_freeze
    cdef int interval_us
    cdef bint use_game_vector
    cdef bint hasAnchor
    cdef int feature
    cdef int detectedSpiked
    cdef int maxSpikes
    cdef double RefreshFrequency
    cdef double lasttime

    def __init__(self, i2c,double RefreshFrequency, int address=0x4A, bint use_game_vector=True,
                 int interval_us=20000, double spike_threshold=20):

        self.use_game_vector = use_game_vector
        self.bno = BNO08X_I2C(i2c, address=address)
        self.feature = BNO_REPORT_GAME_ROTATION_VECTOR if use_game_vector else BNO_REPORT_ROTATION_VECTOR
        self.interval_us = interval_us
        self.spike_threshold = spike_threshold
        self.freeze_count = 0
        self.max_freeze = 50
        self.hasAnchor = False
        self.detectedSpiked = 0
        self.maxSpikes = 3
        self.RefreshFrequency = RefreshFrequency
        self.lasttime = 0

        self.enable_feature()

    cpdef enable_feature(self):
        try:
            self.bno.enable_feature(self.feature, interval=self.interval_us)
        except TypeError:
            self.bno.enable_feature(self.feature)

    cpdef tuple filter_results(self):

        #print(1.0/self.RefreshFrequency > time.time() - self.lasttime)
        if 1.0/self.RefreshFrequency > time.time() - self.lasttime:
            #print("no calculation")
            return None

        self.lasttime = time.time()

        cdef double yaw, pitch, roll
        cdef double raw_q[4]
        cdef object py_quat

        try:
            py_quat = self.bno.game_quaternion if self.use_game_vector else self.bno.quaternion
        except Exception as e:
            print("⚠️ failed quaternion:", e)
            return None

        raw_q[0] = py_quat[0]
        raw_q[1] = py_quat[1]
        raw_q[2] = py_quat[2]
        raw_q[3] = py_quat[3]

        quaternion_to_euler(raw_q, &yaw, &pitch, &roll)

        yaw = math.degrees(yaw)
        pitch = math.degrees(pitch)
        roll = math.degrees(roll)
        if (self.last_euler[0] == yaw and self.last_euler[1] == pitch and self.last_euler[2] == roll):
            self.freeze_count += 1
            if self.freeze_count >= self.max_freeze:
                print("⚠️ Frozen quaternion detected — resetting feature")
                self._reset()
                self.freeze_count = 0
                return None
        else:
            self.freeze_count = 0

        
        if (self.detectedSpiked <= self.maxSpikes and self.hasAnchor and (abs(self.last_euler[0] - yaw) > self.spike_threshold or abs(self.last_euler[1] - pitch) > self.spike_threshold or abs(self.last_euler[2] - roll) > self.spike_threshold)): 
            #print("⚠️ Sudden Spike Detected")
            self.detectedSpiked+=1
            return None
        else:
            self.detectedSpiked=0

        if not self.hasAnchor:
            self.hasAnchor = True
        self.last_euler[0] = yaw
        self.last_euler[1] = pitch
        self.last_euler[2] = roll

    cpdef tuple get_euler(self):
        self.filter_results()
        return (self.last_euler[0],self.last_euler[1],self.last_euler[2])

    cpdef _reset(self):
        self.hasAnchor = False
        try:
            self.enable_feature()
        except Exception as e:
            print("⚠️ Reset failed:", e)
