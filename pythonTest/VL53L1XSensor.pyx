import time
import qwiic_vl53l1x

cdef class VL53L1XSensor:
    cdef object sensor           # Python sensor object
    cdef double lasttime         # last measurement timestamp
    cdef double lastdistance     # cached last distance
    cdef int RefreshFrequency    # times per second to read sensor

    def __init__(self, int RefreshFrequency=40):
        """
        Initialize the VL53L1X sensor using SparkFun Qwiic library.
        """
        self.lasttime = 0.0
        self.lastdistance = 0.0
        self.RefreshFrequency = RefreshFrequency

        self.sensor = qwiic_vl53l1x.QwiicVL53L1X()
        """Initialize the sensor and start ranging."""
        try:
            self.sensor.sensor_init()
            print("Sensor initialized successfully.")
        except Exception as e:
            print(f"Failed to initialize sensor: {e}")

        try:
            self.sensor.start_ranging()
            print("Ranging started.")
        except Exception as e:
            print(f"Failed to start ranging: {e}")
            self.sensor.stop_ranging()

    cpdef double get_distance(self):
        """
        Return the current distance in mm.
        Returns last valid measurement if called too soon or on error.
        """
        cdef double now = time.time()
        cdef double distance

        if not self.sensor:
            return -1.0

        # enforce RefreshFrequency
        if 1.0 / self.RefreshFrequency > now - self.lasttime:
            return self.lastdistance

        self.lasttime = now
        try:
            distance = self.sensor.get_distance()
            if distance < 0:
                distance = self.lastdistance
        except Exception:
            distance = self.lastdistance

        self.lastdistance = distance
        return self.lastdistance

    cpdef close(self):
        """Stop ranging and close the sensor cleanly."""
        if self.sensor:
            try:
                self.sensor.stop_ranging()
                print("✅ Sensor stopped and closed.")
            except Exception as e:
                print("❌ Error closing sensor:", e)
            self.sensor = None
