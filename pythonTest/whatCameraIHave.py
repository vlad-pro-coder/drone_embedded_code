from picamera2 import Picamera2

# Create camera instance
picam2 = Picamera2()

# List available cameras
cameras = Picamera2.global_camera_info()
print("Detected cameras:")
for idx, cam in enumerate(cameras):
    print(f"{idx}: {cam}")

# Open the first camera (usually index 0)
picam2 = Picamera2(0)
config = picam2.create_still_configuration()
print("Current sensor configuration:")
print(config)
