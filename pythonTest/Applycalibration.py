import cv2
import numpy as np

# --- Paths ---
image_path = "calib_00.jpg"        # path to your image
calib_path = "../hosted_site_code/camera_calibration.npz"       # path to your saved K and D
output_path = "undistorted1.jpg"             # path to save undistorted image

# --- Load image ---
img = cv2.imread(image_path)
if img is None:
    raise FileNotFoundError(f"Image not found: {image_path}")

h, w = img.shape[:2]

# --- Load calibration ---
calib = np.load(calib_path)
K = calib["K"]   # intrinsic matrix
D = calib["D"]   # distortion coefficients

# --- Compute undistortion maps ---
new_K, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0, (w, h))
map1, map2 = cv2.initUndistortRectifyMap(K, D, None, new_K, (w, h), cv2.CV_16SC2)

# --- Apply undistortion ---
undistorted = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

# --- Crop to valid region (optional) ---
x, y, w_roi, h_roi = roi
undistorted_cropped = undistorted[y:y+h_roi, x:x+w_roi]

# --- Save the undistorted image ---
cv2.imwrite(output_path, undistorted_cropped)
print(f"Undistorted image saved to: {output_path}")
