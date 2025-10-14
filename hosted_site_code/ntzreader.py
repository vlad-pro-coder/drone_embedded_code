import numpy as np

# Load the .npz file
data = np.load("camera_calibration.npz")

# List all arrays
print("Arrays in file:", data.files)

# Access the camera matrix (K) and distortion coefficients (D)
K = data["K"]
D = data["D"]

print("Camera matrix (K):\n", K)
print("Distortion coefficients (D):\n", D)

fx = K[0, 0]
fy = K[1, 1]
cx = K[0, 2]
cy = K[1, 2]

print("fx:", fx, "fy:", fy, "cx:", cx, "cy:", cy)

fx = K[0, 0]  # from your camera matrix
fy = K[1, 1]
sensor_width = 416  # e.g., 1920
sensor_height = 416  # e.g., 1080

fov_x = 2 * np.degrees(np.arctan(sensor_width / (2 * fx)))
fov_y = 2 * np.degrees(np.arctan(sensor_height / (2 * fy)))

print("Horizontal FOV (deg):", fov_x)
print("Vertical FOV (deg):", fov_y)

