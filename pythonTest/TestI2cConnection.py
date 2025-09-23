import board
import busio
import time

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

# Wait until the bus is ready
while not i2c.try_lock():
    pass

# Scan for devices
devices = i2c.scan()
i2c.unlock()

if devices:
    print("Found I2C devices at addresses:", [hex(d) for d in devices])
else:
    print("No I2C devices found")