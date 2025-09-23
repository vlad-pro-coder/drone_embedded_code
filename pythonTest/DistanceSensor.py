import time
import VL53L1X

class VL53L1XSensor:
    def __init__(self, i2c_bus=1, i2c_address=0x29, timing_ms=50):
        """
        Initialize the VL53L1X sensor.
        - i2c_bus: I2C bus number (default 1)
        - i2c_address: sensor I2C address (default 0x29)
        - timing_ms: measurement timing in milliseconds
        """
        self.tof = VL53L1X.VL53L1X(i2c_bus=i2c_bus, i2c_address=i2c_address)
        try:
            self.tof.open()
            self.tof.start_ranging(timing_ms)
            self.timing = max(timing_ms, 20)  # enforce minimum timing
            print("✅ Sensor initialized and ranging started.")
        except Exception as e:
            print("❌ Failed to initialize sensor:", e)
            self.tof = None

    def get_distance(self):
        """
        Get the current distance in millimeters.
        Returns -1 if the sensor is not ready.
        """
        if not self.tof:
            return -1
        distance = self.tof.get_distance()
        # enforce timing delay between measurements
        time.sleep(self.timing / 1000.0)
        return distance

    def close(self):
        """Stop ranging and close the sensor cleanly."""
        if self.tof:
            try:
                self.tof.stop_ranging()
                self.tof.close()
                print("✅ Sensor stopped and closed.")
            except Exception as e:
                print("❌ Error closing sensor:", e)
            self.tof = None

if __name__ == "__main__":
    sensor = VL53L1XSensor(timing_ms=200)
    for i in range(10):
        dist = sensor.get_distance()
        print(f"Distance: {dist} mm")
    sensor.close()
