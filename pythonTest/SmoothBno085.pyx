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
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR, BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE
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
    cdef double last_gyro[3]
    cdef double spike_threshold_rotation
    cdef double spike_threshold_gyro
    cdef int freeze_count
    cdef int max_freeze
    cdef int interval_us
    cdef bint use_game_vector
    cdef bint hasAnchor_rotation
    cdef bint hasAnchor_gyro
    cdef int feature
    cdef int detectedSpiked_rotation
    cdef int detectedSpiked_gyro
    cdef int maxSpikes
    cdef double RefreshFrequency
    cdef double lasttime
    cdef double TWO_PI
    cdef double minimumyaw
    cdef double minimumpitch
    cdef double minimumroll

    def __init__(self, i2c,double RefreshFrequency, int address=0x4A, bint use_game_vector=True,
                 int interval_us=20000, double spike_threshold_rotation=30, double spike_threshold_gyro=45):

        self.use_game_vector = use_game_vector
        self.bno = BNO08X_I2C(i2c, address=address, debug = False)
        self.interval_us = interval_us
        self.spike_threshold_rotation = spike_threshold_rotation
        self.spike_threshold_gyro = spike_threshold_gyro
        self.freeze_count = 0
        self.max_freeze = 30
        self.hasAnchor_rotation = False
        self.hasAnchor_gyro = False
        self.detectedSpiked_gyro = 0
        self.detectedSpiked_rotation = 0
        #self.maxSpikes = 20
        self.RefreshFrequency = RefreshFrequency
        self.lasttime = 0
        self.TWO_PI = 2.0 * M_PI
        self.minimumyaw = 1e9;
        self.minimumpitch = 1e9;
        self.minimumroll = 1e9;
        #sys.stdout = open("imu_log.txt", "w")

        self.__enable_feature()
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

    cpdef __enable_imu(self):
        feature = BNO_REPORT_GAME_ROTATION_VECTOR if self.use_game_vector else BNO_REPORT_ROTATION_VECTOR
        try:
            self.bno.enable_feature(feature, interval=self.interval_us)
        except TypeError:
            self.bno.enable_feature(feature)

    cpdef __enable_gyro(self):
        try:
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE, interval=self.interval_us)
        except TypeError:
            self.bno.enable_feature(BNO_REPORT_GYROSCOPE)

    cpdef __enable_feature(self):
        self.__enable_imu()
        self.__enable_gyro()

    cpdef double __getAngleDifference(self, double target, double current):
        diff = fmod(target - current + 180.0, 360.0)
        if diff < 0:
            diff += 360.0
        return diff - 180.0

    cpdef double __speeddif(self,double a, double b):
        return a - b

    cpdef void __filterVelocities(self):
        cdef object py_gyro

        cdef double yaw_vel, pitch_vel, roll_vel

        try:
            py_gyro = self.bno.gyro
        except Exception as e:
            #print("⚠️ failed quaternion:", e)
            return

        roll_vel = math.degrees(py_gyro[0])
        pitch_vel = math.degrees(py_gyro[1])
        yaw_vel = math.degrees(py_gyro[2])

        if (self.last_gyro[0] == pitch_vel and self.last_gyro[1] == roll_vel and self.last_gyro[2] == yaw_vel):
            self.freeze_count += 1
            if self.freeze_count >= self.max_freeze:
                print("⚠️ Frozen gyroscope detected — resetting feature")
                self.__reset()
                self.freeze_count = 0
                return
        else:
            self.freeze_count = 0

        cdef double diff_yaw_vel = self.__speeddif(self.last_gyro[0],yaw_vel)
        cdef double diff_pitch_vel = self.__speeddif(self.last_gyro[1],pitch_vel)
        cdef double diff_roll_vel = self.__speeddif(self.last_gyro[2],roll_vel)
        if (self.hasAnchor_gyro and (abs(diff_yaw_vel) > self.spike_threshold_gyro or abs(diff_pitch_vel) > self.spike_threshold_gyro or abs(diff_roll_vel) > self.spike_threshold_gyro)): 
            #print("⚠️ Sudden Spike Detected")
            self.detectedSpiked_gyro+=1
            return
        else:
            self.hasAnchor_gyro=False

        if not self.hasAnchor_gyro:
            self.hasAnchor_gyro = True
        self.last_gyro[0] = yaw_vel
        self.last_gyro[1] = pitch_vel
        self.last_gyro[2] = roll_vel

    cpdef void __filterAngles(self):

        cdef double yaw, pitch, roll
        cdef double raw_q[4]
        cdef object py_quat

        try:
            py_quat = self.bno.game_quaternion if self.use_game_vector else self.bno.quaternion
        except Exception as e:
            #print("⚠️ failed quaternion:", e)
            return

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
                print("⚠️ Frozen quaternion detected — resetting feature")
                self.__reset()
                self.freeze_count = 0
                return
        else:
            self.freeze_count = 0

        cdef double yawdiff = self.__getAngleDifference(yaw,self.last_euler[0])
        cdef double pitchdiff = self.__getAngleDifference(pitch,self.last_euler[1])
        cdef double rolldiff = self.__getAngleDifference(roll,self.last_euler[2])
        
        #self.minimumyaw = min(self.minimumyaw,yaw)
        #self.minimumpitch = min(self.minimumpitch,pitch)
        #self.minimumroll = min(self.minimumroll,roll)
        #print(" min Yaw: ", self.minimumyaw , " min Pitch: ",self.minimumpitch, " min Roll: " ,{self.minimumroll})
        #print(self.detectedSpiked)
        #print(" Yaw: ", yaw , " Pitch: ",pitch, " Roll: " ,roll)
        if (self.hasAnchor_rotation and (abs(yawdiff) > self.spike_threshold_rotation or abs(pitchdiff) > self.spike_threshold_rotation or abs(rolldiff) > self.spike_threshold_rotation)): 
            #print("⚠️ Sudden Spike Detected")
            self.detectedSpiked_rotation+=1
            return
        else:
            self.hasAnchor_rotation=False

        if not self.hasAnchor_rotation:
            self.hasAnchor_rotation = True
        self.last_euler[0] = yaw
        self.last_euler[1] = pitch
        self.last_euler[2] = roll

    cpdef void update(self):
        #print(1.0/self.RefreshFrequency > time.time() - self.lasttime)
        if 1.0/self.RefreshFrequency > time.time() - self.lasttime:
            #print("no calculation")
            return

        self.lasttime = time.time()
        self.__filterAngles()
        self.__filterVelocities()

    cpdef tuple getAngle(self):
        return (self.last_euler[0],self.last_euler[1],self.last_euler[2])
    cpdef tuple getVelocity(self):
        return (self.last_gyro[0],self.last_gyro[1],self.last_gyro[2])


    cpdef __reset(self):
        self.hasAnchor_rotation = False
        self.hasAnchor_gyro = False
        try:
            self.enable_feature()
        except Exception as e:
            #print("⚠️ Reset failed:", e)
            return None

    
