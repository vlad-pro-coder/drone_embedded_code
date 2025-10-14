import numpy as np
cimport numpy as np
cimport cython
from picamera2 import Picamera2
import cv2
import time
import os

cdef class RaspGSCamera:
    cdef object picam2
    cdef np.ndarray K
    cdef np.ndarray D
    cdef np.ndarray map1
    cdef np.ndarray map2
    cdef tuple roi
    cdef int width
    cdef int height
    cdef bint started

    def __cinit__(self, int width=1456, int height=1088, const char* calib_file = b"camera_calibration.npz"):
        # Minimal work here, real init in __init__ (to allow Python exceptions)
        self.width = width
        self.height = height
        self.started = False
        self.picam2 = None
        self.K = None
        self.D = None
        self.map1 = None
        self.map2 = None
        self.roi = (0, 0, width, height)

    def __init__(self, int width=1456, int height=1088, calib_file="camera_calibration.npz"):
        # Keep Python-level initialization here (can raise)
        if not os.path.exists(calib_file):
            raise FileNotFoundError(f"Calibration file not found: {calib_file}")

        data = np.load(calib_file)
        self.K = np.asarray(data["K"], dtype=np.float64)
        self.D = np.asarray(data["D"], dtype=np.float64)

        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (width, height), "format": "BGR888"}
        )
        config["controls"] = {
            "NoiseReductionMode": 2,
            "AwbEnable": True,
            "AeEnable": True,
            "Sharpness": 1.0,
            "Contrast": 1.0,
        }
        self.picam2.configure(config)
        self.picam2.start()
        time.sleep(1.0)  # warm-up
        self.started = True

        # Precompute undistort/remap maps for this resolution
        h = height
        w = width
        new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 0, (w, h))
        # map1, map2 are cv2-compatible arrays for remap
        map1, map2 = cv2.initUndistortRectifyMap(self.K, self.D, None, new_K, (w, h), cv2.CV_16SC2)
        # store as contiguous arrays
        self.map1 = np.ascontiguousarray(map1)
        self.map2 = np.ascontiguousarray(map2)
        self.roi = roi
        self.width = width
        self.height = height

    def getIntrinsics(self):
        """Return fx, fy, cx, cy as Python tuple"""
        # K expected shape (3,3)
        return (float(self.K[0,0]), float(self.K[1,1]), float(self.K[0,2]), float(self.K[1,2]))

    def capture_mat(self):
        """
        Capture and return an undistorted, cropped BGR image as a NumPy array.
        This returns a view (copy may be created by remap).
        """
        if not self.started:
            raise RuntimeError("Camera not started")

        # GIL required to call picamera2 and OpenCV
        img = self.picam2.capture_array()  # BGR888 numpy array

        # Remap/undistort in-place-ish (remap returns new array)
        undistorted = cv2.remap(img, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        x, y, w_roi, h_roi = self.roi
        cropped = undistorted[y:y+h_roi, x:x+w_roi]

        # ensure contiguous array of type uint8
        return img

    def capture_jpeg(self, int quality=100):
        """Return JPEG bytes of the undistorted image."""
        frame_bgr = self.capture_mat()
        # convert to RGB for encoding if desired, but many pipelines accept BGR as well.
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        ok, buf = cv2.imencode(".jpg", frame_rgb, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            return None
        return buf.tobytes()

    def stop(self):
        if self.started and self.picam2 is not None:
            try:
                self.picam2.stop()
            except Exception:
                pass
            self.started = False
