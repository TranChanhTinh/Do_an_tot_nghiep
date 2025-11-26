#!/usr/bin/env python3
import cv2
import numpy as np
import time
import torch
import struct

ENABLE_UART = True
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUD = 115200

try:
    if ENABLE_UART:
        import serial
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.5)
    else:
        ser = None
except Exception as e:
    print(f"[WARN] Không mở được serial: {e}")
    ser = None
    ENABLE_UART = False

from picamera2 import Picamera2
from ultralytics import YOLO

# ================= CONFIG ================= #
LANE_MODEL_PATH = "/home/gino/Downloads/Do_an_tot_nghiep/yolo11n_seg.pt"
SIGN_MODEL_PATH = "/home/gino/Downloads/Do_an_tot_nghiep/sign.pt"

MODEL_INPUT_WIDTH = 320
MODEL_INPUT_HEIGHT = 320
CONFIDENCE_THRESHOLD = 0.35
SIGN_CONFIDENCE = 0.45
MASK_THRESHOLD = 0.5
DEVICE = 'cpu'

STEERING_CENTER = 90
STEERING_MIN = 40
STEERING_MAX = 140
SMOOTH_FACTOR = 0.3
DEADBAND = 1
MAX_STEER_OFFSET_DEG = 60
LOSS_TIMEOUT = 1.0
SHORT_LOSS_HOLD = 0.4

speed_motor_requested = 0
previous_angle = STEERING_CENTER
last_lane_time = time.time()
current_rpm = 0  # RPM giữ tốc độ biển
rpm_scale_factor = 1.0
manual_stop = False

lane_lost_stop = False   # <<< ADDED — cờ dừng khi mất lane


# ================= HỖ TRỢ HÀM ================= #
def transform_speed(vel):
    f = vel / 10
    rpm = (f * 30 * 2.85) / (3.14 * 0.03 * 3.6)
    return int(round(rpm))

def process_lane_mask(results, h_orig, w_orig, mask_threshold=MASK_THRESHOLD):
    mask = np.zeros((h_orig, w_orig), dtype=np.uint8)
    if results is None:
        return mask
    try:
        if getattr(results, 'masks', None) is None:
            return mask
        masks_data = results.masks.data
        for i, cls in enumerate(results.boxes.cls):
            label = results.names[int(cls)]
            if label == "lane":
                mask_np = masks_data[i].cpu().float().numpy()
                resized = cv2.resize(mask_np, (w_orig, h_orig))
                mask[resized > mask_threshold] = 255
    except Exception:
        pass
    return mask

def get_lane_centroid_x(mask):
    h, w = mask.shape
    roi = mask[int(h * 0.5):, :]
    M = cv2.moments(roi)
    if M["m00"] == 0:
        return None
    return int(M["m10"] / M["m00"])

def calculate_steering_from_centroid(cx, frame_width):
    offset = (cx - frame_width // 2) / (frame_width // 2)
    steer = STEERING_CENTER + int(offset * MAX_STEER_OFFSET_DEG)
    return int(np.clip(steer, STEERING_MIN, STEERING_MAX))

def smooth_and_deadband(new_angle, prev_angle):
    smooth = int(prev_angle * SMOOTH_FACTOR + new_angle * (1 - SMOOTH_FACTOR))
    return prev_angle if abs(smooth - prev_angle) <= DEADBAND else smooth


# ================= INIT MODELS + CAMERA ================= #
print("Loading models...")
lane_model = YOLO(LANE_MODEL_PATH).to(DEVICE).eval()
sign_model = YOLO(SIGN_MODEL_PATH).to(DEVICE).eval()
print("Models ready.")

piCam = Picamera2()
camera_config = piCam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
piCam.configure(camera_config)
piCam.start()
time.sleep(0.2)

prev_time = 0
print("Running... (press 'q' to quit)")

# ================= MAIN LOOP ================= #
try:
    while True:
        frame = piCam.capture_array()
        if frame is None:
            time.sleep(0.01)
            continue
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
        h, w = frame.shape[:2]

        # ---------- LANE DETECTION ---------- #
        lane_result = None
        try:
            with torch.no_grad():
                for r in lane_model.predict(frame, imgsz=MODEL_INPUT_WIDTH,
                                            conf=CONFIDENCE_THRESHOLD, device=DEVICE,
                                            stream=True, verbose=False):
                    lane_result = r
                    break
        except TypeError:
            with torch.no_grad():
                lane_result = lane_model(frame, imgsz=MODEL_INPUT_WIDTH,
                                         conf=CONFIDENCE_THRESHOLD, device=DEVICE, verbose=False)

        lane_mask = process_lane_mask(lane_result, h, w)
        cx = get_lane_centroid_x(lane_mask)

        steer_by_mask = calculate_steering_from_centroid(cx, w) if cx is not None else None

        if steer_by_mask is not None:
            steering_angle = smooth_and_deadband(steer_by_mask, previous_angle)
            previous_angle = steering_angle
            last_lane_time = time.time()
        else:
            # <<< MODIFIED — thay toàn bộ xử lý mất lane bằng dừng xe
            lane_lost_stop = True
            speed_motor_requested = 0
            steering_angle = STEERING_CENTER

        # ---------- TRAFFIC SIGN DETECTION ----------
        sign_results = None
        try:
            with torch.no_grad():
                for rs in sign_model.predict(frame, imgsz=MODEL_INPUT_WIDTH,
                                             conf=SIGN_CONFIDENCE, device=DEVICE,
                                             stream=True, verbose=False):
                    sign_results = rs
                    break
        except TypeError:
            with torch.no_grad():
                sign_results = sign_model(frame, imgsz=MODEL_INPUT_WIDTH,
                                          conf=SIGN_CONFIDENCE, device=DEVICE, verbose=False)

        # ---------- PROCESS TRAFFIC SIGNS ----------
        stop_detected = False
        speed_rpm_detected = None

        if sign_results and not manual_stop:
            try:
                for box in sign_results.boxes:
                    cls = int(box.cls[0]) if hasattr(box.cls, '__len__') else int(box.cls)
                    name = sign_results.names[cls].lower() if cls < len(sign_results.names) else str(cls)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0]) if hasattr(box.conf, '__len__') else float(box.conf)

                    if "stop" in name:
                        stop_detected = True
                        break
                    elif "speed20" in name:
                        speed_rpm_detected = 200
                    elif "speed30" in name:
                        speed_rpm_detected = 300
                    elif "speed50" in name:
                        speed_rpm_detected = 400
            except Exception:
                pass

        # ---------- APPLY TRAFFIC SIGN ACTION ---------- #
        if manual_stop or lane_lost_stop:   # <<< MODIFIED — thêm lane_lost_stop
            rpm_cmd = 0
            current_rpm = 0
            speed_motor_requested = 0

        else:
            if stop_detected:
                speed_motor_requested = 0
                current_rpm = 0

            elif speed_rpm_detected is not None:
                current_rpm = speed_rpm_detected
                speed_motor_requested = current_rpm

            rpm_cmd = current_rpm if current_rpm > 0 else transform_speed(speed_motor_requested)

            angle_diff = abs(steering_angle - STEERING_CENTER)
            if angle_diff > 8:
                rpm_cmd = int(rpm_cmd * 0.8)
            else:
                rpm_cmd = int(rpm_cmd * 1.0)


        # ---------- OVERLAY ----------
        overlay = frame.copy()

        if sign_results:
            try:
                for box in sign_results.boxes:
                    coords = box.xyxy[0].cpu().numpy() if hasattr(box.xyxy[0], 'cpu') else box.xyxy[0]
                    x1, y1, x2, y2 = map(int, coords)
                    cls = int(box.cls[0]) if hasattr(box.cls, '__len__') else int(box.cls)
                    name = sign_results.names[cls] if cls < len(sign_results.names) else str(cls)
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    cv2.putText(overlay, name, (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            except Exception:
                pass

        mask_color = np.zeros_like(frame)
        mask_color[lane_mask == 255] = [0, 255, 0]
        overlay = cv2.addWeighted(overlay, 1.0, mask_color, 0.5, 0)

        mid_x = w // 2
        cv2.line(overlay, (mid_x, h), (mid_x, h // 2), (0, 0, 255), 2)
        if cx is not None:
            cv2.circle(overlay, (cx, int(h * 0.75)), 6, (255, 0, 0), -1)
            cv2.line(overlay, (cx, int(h * 0.75)), (mid_x, int(h * 0.75)), (255, 0, 0), 1)

        # ---------- HUD DISPLAY ----------
        fps = 1 / (time.time() - prev_time) if prev_time else 0
        prev_time = time.time()

        cv2.putText(overlay, f"FPS: {fps:.1f}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(overlay, f"Steer: {steering_angle}", (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(overlay, f"SpeedReq: {speed_motor_requested} RPM:{rpm_cmd}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

        cv2.imshow("Lane + Traffic Sign - Merged", overlay)

        # ---------- UART SEND ----------
        if ENABLE_UART and ser:
            try:
                ser.write(b'<' + struct.pack('<ii', int(steering_angle), int(rpm_cmd)) + b'>')
                ser.flush()
            except Exception as e:
                print("[WARN] UART write error:", e)

        key = cv2.waitKey(1) & 0xFF
        signal_motor(key)

        if key == ord('q'):
            if ENABLE_UART and ser:
                ser.write(b'<' + struct.pack('<ii', STEERING_CENTER, 0) + b'>')
                ser.flush()
            print("User exit")
            break

except KeyboardInterrupt:
    print("\nInterrupted by user. Exiting...")
finally:
    if ENABLE_UART and ser:
        ser.close()
    try:
        piCam.stop()
    except Exception:
        pass
    cv2.destroyAllWindows()
    print("Clean exit.")
