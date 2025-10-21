from picamera2 import Picamera2
import time
import cv2
import numpy as np
from RaspGSCamera import RaspGSCamera

camera = RaspGSCamera()

frame_rgb = camera.capture_mat()

# Save the image
cv2.imwrite("frame_rgb.jpg", frame_rgb)
print("Saved frame_rgb.jpg (resized with interpolation)")
