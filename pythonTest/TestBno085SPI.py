# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

import time
import board
import busio
from digitalio import DigitalInOut, Direction
from adafruit_bno08x.spi import BNO08X_SPI

# Initialize SPI bus
spi = busio.SPI(board.SCK, MISO=board.MISO, MOSI=board.MOSI)
# Chip Select (CS) pin
cs = DigitalInOut(board.CE0)  # BCM 17
cs.direction = Direction.OUTPUT

# Interrupt pin
int_pin = DigitalInOut(board.D22)  # BCM 22
int_pin.direction = Direction.INPUT

# Optional Reset pin (can be None if not used)
reset_pin = DigitalInOut(board.D27)  # BCM 27
reset_pin.direction = Direction.OUTPUT
reset_pin.value = True
time.sleep(0.1)  # wait 100ms after reset

# Initialize the BNO085 over SPI
bno = BNO08X_SPI(spi, cs, int_pin, reset_pin, 1000000, debug=False)

while True:
    print("sigma")
    try:
        quat = bno.quaternion
        print("Rotation Vector Quaternion:")
        print(f"I: {quat.i:.6f}  J: {quat.j:.6f}  K: {quat.k:.6f}  Real: {quat.real:.6f}")
    except Exception as e:
        print("⚠️ Error reading quaternion:", e)
    time.sleep(0.5)