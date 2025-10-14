# smoothed_bno08x.pyx
# ------------------------------
# Fast Cython version: all quaternions in C arrays (double[4])
# ------------------------------

import math
from cpython cimport bool
cimport cython
from libc.math cimport fmod, fabs, M_PI

# Python sensor interface stays as object
import board
import busio
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C
import time
import sys

# ------------------------------
# Helper C functions
# ------------------------------

@cython.cdivision(True)
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
    cdef double TWO_PI
    cdef double minimumyaw
    cdef double minimumpitch
    cdef double minimumroll

    def __init__(self, i2c,double RefreshFrequency, int address=0x4A, bint use_game_vector=True,
                 int interval_us=20000, double spike_threshold=30):

        self.use_game_vector = use_game_vector
        self.bno = BNO08X_I2C(i2c, address=address, debug = False)
        self.feature = BNO_REPORT_GAME_ROTATION_VECTOR if use_game_vector else BNO_REPORT_ROTATION_VECTOR
        self.interval_us = interval_us
        self.spike_threshold = spike_threshold
        self.freeze_count = 0
        self.max_freeze = 50
        self.hasAnchor = False
        self.detectedSpiked = 0
        #self.maxSpikes = 20
        self.RefreshFrequency = RefreshFrequency
        self.lasttime = 0
        self.TWO_PI = 2.0 * M_PI
        self.minimumyaw = 1e9;
        self.minimumpitch = 1e9;
        self.minimumroll = 1e9;
        sys.stdout = open("imu_log.txt", "w")

        self.enable_feature()
    cdef void __quaternion_to_euler(self,double q[4], double* yaw, double* pitch, double* roll):
        cdef double w = q[3]
        cdef double x = q[0]
        cdef double y = q[1]
        cdef double z = q[2]

        # Pitch
        cdef double sinp = 2.0 * (w*y - z*x)
        if sinp >= 1.0:
            pitch[0] = M_PI/2   # clamp
        elif sinp <= -1.0:
            pitch[0] = -M_PI/2  # clamp
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

    cpdef enable_feature(self):
        try:
            self.bno.enable_feature(self.feature, interval=self.interval_us)
        except TypeError:
            self.bno.enable_feature(self.feature)

    cpdef double __getAngleDifference(self, double target, double current):
        diff = fmod(target - current + 180.0, 360.0)
        if diff < 0:
            diff += 360.0
        return diff - 180.0

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
            #print("⚠️ failed quaternion:", e)
            return None

        raw_q[0] = py_quat[0]
        raw_q[1] = py_quat[1]
        raw_q[2] = py_quat[2]
        raw_q[3] = py_quat[3]

        self.__quaternion_to_euler(raw_q, &yaw, &pitch, &roll)

        yaw = math.degrees(yaw)
        pitch = math.degrees(pitch)
        roll = math.degrees(roll)
        if (self.last_euler[0] == yaw and self.last_euler[1] == pitch and self.last_euler[2] == roll):
            self.freeze_count += 1
            if self.freeze_count >= self.max_freeze:
                #print("⚠️ Frozen quaternion detected — resetting feature")
                self._reset()
                self.freeze_count = 0
                return None
        else:
            self.freeze_count = 0

        cdef double yawdiff = self.__getAngleDifference(yaw,self.last_euler[0])
        cdef double pitchdiff = self.__getAngleDifference(pitch,self.last_euler[1])
        cdef double rolldiff = self.__getAngleDifference(roll,self.last_euler[2])
        
        self.minimumyaw = min(self.minimumyaw,yaw)
        self.minimumpitch = min(self.minimumpitch,pitch)
        self.minimumroll = min(self.minimumroll,roll)
        print(" min Yaw: ", self.minimumyaw , " min Pitch: ",self.minimumpitch, " min Roll: " ,{self.minimumroll})
        print(self.detectedSpiked)
        print(" Yaw: ", yaw , " Pitch: ",pitch, " Roll: " ,roll)
        if (self.hasAnchor and (abs(yawdiff) > self.spike_threshold or abs(pitchdiff) > self.spike_threshold or abs(rolldiff) > self.spike_threshold)): 
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
            #print("⚠️ Reset failed:", e)
            return None
