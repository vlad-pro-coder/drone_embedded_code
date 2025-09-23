import spidev
import time
import board
from digitalio import DigitalInOut, Direction

spi = spidev.SpiDev()
spi.open(0, 0)          # bus 0, device 0 (/dev/spidev0.0)
spi.max_speed_hz = 1000000  # 1 MHz is safe for testing
spi.mode = 0b11  # Mode 3 for BNO085

#cs = DigitalInOut(board.CE0)  # BCM 17
#cs.direction = Direction.OUTPUT
#
#cs.value = False
#
#int_pin = DigitalInOut(board.D22)  # BCM 22
#int_pin.direction = Direction.INPUT

try:
    while True:
        resp = spi.xfer2([0x00])   # send dummy byte, read response
        print("Read:", resp)
        time.sleep(0.5)
except KeyboardInterrupt:
    spi.close()
