from flask import Flask, Response, render_template_string, redirect, url_for, jsonify
from picamera2 import Picamera2
import sys
sys.path.append("../pythonTest")
import cv2
import numpy as np
import os
from RaspGSCamera import RaspGSCamera
import time
from ClientPhotoSender import ClientPhotoSender
from threading import Thread

app = Flask(__name__)
ClientProcessImages = ClientPhotoSender("192.168.1.10")
client_thread = False
client_running = False

# Directory to save calibration images
CALIB_DIR = "calib_images"
os.makedirs(CALIB_DIR, exist_ok=True)

# Chessboard parameters
CHESSBOARD_SIZE = (9, 6)
SQUARE_SIZE = 2.0  # arbitrary units

# HTML template for calibration page
CALIB_TEMPLATE = """
<!doctype html>
<title>Camera Calibration</title>
<h1>Camera Calibration</h1>
<form action="{{ url_for('start_calibration') }}" method="post">
    <button type="submit">Start Calibration</button>
</form>
"""

def gen_frames():
    while True:
        frame_bytes = ClientProcessImages.camera.capture_jpeg()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    global client_running

    if client_running:
        client_thread.stop()
        client_running = False

    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/calibration')
def calibration():
    return render_template_string(CALIB_TEMPLATE)

@app.route('/start_calibration', methods=['POST'])
def start_calibration():
    # Capture 15 images
    time.sleep(2)
    captured_files = []
    for i in range(45):
        frame = ClientProcessImages.camera.capture_mat()  # get as NumPy array
        filename = os.path.join(CALIB_DIR, f"calib_{i:02d}.jpg")
        cv2.imwrite(filename, frame)
        captured_files.append(filename)
        time.sleep(0.1)  # short delay between captures

    # Run calibration
    objpoints = []
    imgpoints = []

    # Prepare object points
    objp = np.zeros((CHESSBOARD_SIZE[0]*CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE

    for fname in captured_files:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
        if ret:
            imgpoints.append(corners)
            objpoints.append(objp)

    if len(objpoints) < 5:
        return "Calibration failed: not enough valid images"

    ret, K, D, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # Save calibration results
    np.savez("camera_calibration.npz", K=K, D=D)

    return f"Calibration done! Saved K and D to camera_calibration.npz. RMS error: {ret:.4f}"

def draw_boxes(image, boxes):
    """Draw bounding boxes on the image."""
    for box in boxes:
        x, y, w, h, score, class_id = box
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        label = f"{class_id}: {score:.2f}"
        cv2.putText(image, label, (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return image

@app.route('/detection_test')
def detect_frame():
    global client_thread, client_running

    if not client_running:
        def run_client():
            global client_running
            client_running = True
            try:
                ClientProcessImages.start_sending_packets()
            finally:
                client_running = False

        client_thread = Thread(target=run_client, daemon=True)
        client_thread.start()
        return "Client started. <a href='/'>Go back</a>"

    return jsonify({"results": ClientProcessImages.result})
    

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8000)
