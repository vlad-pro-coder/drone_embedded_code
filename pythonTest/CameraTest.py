from picamera2 import Picamera2
import time
import cv2
import numpy as np

# Initialize Picamera2
data = np.load("../hosted_site_code/camera_calibration.npz")
K = data["K"]  # 3x3 camera matrix
D = data["D"]  # distortion coefficients
picam2 = Picamera2()
config = picam2.create_still_configuration(
            main={"size": (416, 416), "format": "BGR888"}
        )
config["controls"] = {
            "NoiseReductionMode": 2,  # 2 = High quality
            "AwbEnable": True,
            "AeEnable": True,
            "Sharpness": 1.0,
            "Contrast": 1.0,
            "ExposureValue": -0.3,
        }

picam2.configure(config)
picam2.start()
time.sleep(1)

# Capture full-resolution BGR frame
frame_bgr = picam2.capture_array()
frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

#if target_size:
#    frame_rgb = cv2.resize(frame_rgb, target_size, interpolation=cv2.INTER_AREA)
h, w = frame_rgb.shape[:2]
new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0, (w, h))

undistorted = cv2.undistort(frame_rgb, K, D, None, new_K)

x, y, w_roi, h_roi = roi
undistorted_cropped = undistorted[y:y+h_roi, x:x+w_roi]

# Save the image
cv2.imwrite("frame_rgb.jpg", undistorted)
print("Saved frame_rgb.jpg (resized with interpolation)")

# Stop the camera
picam2.stop()
