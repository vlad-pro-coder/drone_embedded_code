# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time

import board
import busio
import digitalio

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_RAW_GYROSCOPE,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_RAW_ACCELEROMETER
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA, frequency = 800000)
bno = BNO08X_I2C(i2c)
REPORT_INTERVAL = 10000

bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR,5000)
bno.enable_feature(BNO_REPORT_GYROSCOPE,5000)
lasttime = 0

int_pin = digitalio.DigitalInOut(board.D20)
int_pin.direction = digitalio.Direction.INPUT

while True:

    try:
        print("frequency",1 / (time.time() - lasttime))
        lasttime = time.time()
        #print("Acceleration:")
        #print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
        #print("")

        #print("Gyro:")
        a,b,c,d = bno.game_quaternion
        #print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
        #print("")
    except Exception as e:
        print("⚠️ Read error:", e)
    