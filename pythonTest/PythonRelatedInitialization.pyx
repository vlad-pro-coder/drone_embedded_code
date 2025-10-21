import board, busio

class PythonRelatedInitializer:
    # public static variable
    i2c = None

    @staticmethod
    def initialize():
        # Initialize I2C bus and store in static variable
        PythonRelatedInitializer.i2c = busio.I2C(board.SCL, board.SDA,frequency=400000)

