import cv2
import time
import numpy as np
from picamera2 import Picamera2

class RaspGSCamera:
    def __init__(self, width=640, height=640):
        data = np.load("camera_calibration.npz")
        self.K = data["K"]  # 3x3 camera matrix
        self.D = data["D"]  # distortion coefficients
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
        time.sleep(1)  # warm-up

    def capture_mat(self):
        """Capture as RGB NumPy array (OpenCV format)."""
        frame_bgr = self.picam2.capture_array()

        #if target_size:
        #    frame_rgb = cv2.resize(frame_rgb, target_size, interpolation=cv2.INTER_AREA)
        h, w = frame_bgr.shape[:2]
        new_K, roi = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), 0, (w, h))

        undistorted = cv2.undistort(frame_bgr, self.K, self.D, None, new_K)

        x, y, w_roi, h_roi = roi
        undistorted_cropped = undistorted[y:y+h_roi, x:x+w_roi]

        return frame_bgr

    def capture_jpeg(self, quality=90):
        """Capture as JPEG bytes (RGB encoded)."""
        frame_bgr = self.capture_mat()
        frame_rgb = cv2.cvtColor(frame_bgr,cv2.COLOR_BGR2RGB)
        ok, buf = cv2.imencode(".jpg", frame_rgb, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        return buf.tobytes() if ok else None

    def stop(self):
        self.picam2.stop()
