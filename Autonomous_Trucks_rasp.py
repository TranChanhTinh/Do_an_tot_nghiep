#!/usr/bin/env python3
import cv2
import numpy as np
import time
import torch
import serial
import struct
import threading
from picamera2 import Picamera2
from ultralytics import YOLO

# ================== CONFIG UART / MODEL ==================
speed_motor_requested = 0
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

PYTORCH_MODEL_PATH = "Auto.pt"
CONFIDENCE_THRESHOLD = 0.35
MASK_THRESHOLD = 0.5
DEVICE = "cpu"

# ================== SERVO CONFIG ==================
STEERING_SENSITIVITY = 60
MAX_STEERING_ANGLE = 35.0
DEAD_ZONE_THRESHOLD = 0.05
STEERING_CENTER = 90
STEERING_MIN = 40
STEERING_MAX = 140

# ================== GLOBAL CAMERA FRAME ==================
latest_frame = None
stop_thread = False

# ================== CAMERA THREAD ==================
def camera_capture_thread(picam):
    global latest_frame, stop_thread
    while not stop_thread:
        latest_frame = picam.capture_array()

# ================== RATIO CALC ==================
def calculate_lane_background_ratios(inverted_binary_mask):
    h, w = inverted_binary_mask.shape
    mid = w // 2

    left = inverted_binary_mask[:, :mid]
    right = inverted_binary_mask[:, mid:]

    bl = np.sum(left == 0)
    wl = np.sum(left == 255)
    left_ratio = bl / wl if wl > 0 else float('inf') if bl > 0 else 0.0

    br = np.sum(right == 0)
    wr = np.sum(right == 255)
    right_ratio = br / wr if wr > 0 else float('inf') if br > 0 else 0.0

    return left_ratio, right_ratio

# ================== SERVO COMPUTE ==================
def calculate_steering_from_ratios(ratio_l, ratio_r):
    if ratio_l == float('inf') and ratio_r == float('inf'):
        steering = 0
    elif ratio_l == float('inf'):
        steering = -MAX_STEERING_ANGLE
    elif ratio_r == float('inf'):
        steering = MAX_STEERING_ANGLE
    else:
        diff = ratio_l - ratio_r
        if abs(diff) < DEAD_ZONE_THRESHOLD:
            steering = 0
        else:
            steering = STEERING_SENSITIVITY * diff
            steering = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering))

    servo_value = int(round(STEERING_CENTER - steering))
    servo_value = max(STEERING_MIN, min(STEERING_MAX, servo_value))
    return servo_value

# ================== SPEED CONTROL ==================
def transform_speed(vel):
    f = vel / 10
    rpm = (f * 30 * 2.85) / (3.14 * 0.03 * 3.6)
    return int(round(rpm))

# ================== LOAD YOLO MODEL ==================
model = YOLO(PYTORCH_MODEL_PATH)

# ================== CAMERA CONFIG ==================
piCam = Picamera2()
config = piCam.create_preview_configuration(main={"format": "RGB888", "size": (256, 192)})
piCam.configure(config)
piCam.start()
time.sleep(0.5)

# Start capture thread
cam_thread = threading.Thread(target=camera_capture_thread, args=(piCam,), daemon=True)
cam_thread.start()

print(" Camera + YOLO lane running... Nhấn Q để thoát")

# ================== MAIN LOOP ==================
show_display = True
frame_count = 0

try:
    while True:
        if latest_frame is None:
            continue
        frame = latest_frame.copy()

        results = model.predict(frame, imgsz=224, verbose=False)

        mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

        if results and results[0].masks is not None:
            for m in results[0].masks.data:
                m = m.cpu().numpy()
                m = cv2.resize(m, (frame.shape[1], frame.shape[0]))
                m = (m > MASK_THRESHOLD).astype(np.uint8) * 255
                mask[m == 255] = 255

        inverted = cv2.bitwise_not(mask)

        # Ratio tính lệch lane
        ratio_left, ratio_right = calculate_lane_background_ratios(inverted)
        steering_angle = calculate_steering_from_ratios(ratio_left, ratio_right)

        key = cv2.waitKey(1) & 0xFF
        signal_motor(key)
        rpm_cmd = transform_speed(speed_motor_requested)

        # Gửi UART
        if ser:
            data = struct.pack('<ii', steering_angle, rpm_cmd)
            ser.write(b'<' + data + b'>')

        frame_count += 1
        if frame_count % 10 == 0:
            ser.flush()

        # Hiển thị
        if show_display:
            annotated = results[0].plot()
            cv2.imshow("YOLO + Lane Mask", annotated)

        print(f"FPS ~{int(piCam.capture_metadata()['FrameDuration'] and 1/(piCam.capture_metadata()['FrameDuration']/1e6)) if 'FrameDuration' in piCam.capture_metadata() else '?'} | Left={ratio_left:.2f} Right={ratio_right:.2f} Servo={steering_angle} Speed={speed_motor_requested}")

        if key == ord('q'):
            data = struct.pack('<ii', 90, 0)
            ser.write(b'<' + data + b'>')
            ser.flush()
            break

        elif key == ord('v'):  # Toggle hiển thị để tiết kiệm FPS
            show_display = not show_display
            if not show_display:
                cv2.destroyAllWindows()

except KeyboardInterrupt:
    print("\n Dừng (Ctrl+C)")
finally:
    stop_thread = True
    time.sleep(0.2)
    piCam.stop()
    cv2.destroyAllWindows()
    print(" Giải phóng camera.")
