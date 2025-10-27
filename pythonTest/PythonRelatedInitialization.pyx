import board, busio, digitalio, time
class PythonRelatedInitializer:
    # public static variable
    i2c = None

    @staticmethod
    def initialize():
        reset_pin = digitalio.DigitalInOut(board.D21)
        reset_pin.direction = digitalio.Direction.OUTPUT
        # Initialize I2C bus and store in static variable
        reset_pin.value = False
        time.sleep(0.02)
        reset_pin.value = True
        time.sleep(0.5)
        PythonRelatedInitializer.i2c = busio.I2C(board.SCL, board.SDA,frequency=800000)

