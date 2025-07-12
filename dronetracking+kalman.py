import cv2
from ultralytics import YOLO
import serial
import time
import numpy as np

model = YOLO(r"C:\Users\aakas\OneDrive\Desktop\drone_project\runs\detect\train11\weights\best.pt")

try:
    ser = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(2)
    print("Serial connected to COM3")
except:
    print("Could not connect to Arduino on COM3")
    exit()

class SimpleKalman:
    def __init__(self, process_noise=1e-2, measurement_noise=1e-1, error=1.0):
        self.x = None
        self.P = error
        self.Q = process_noise
        self.R = measurement_noise

    def update(self, measurement):
        if self.x is None:
            self.x = measurement
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)
        return self.x

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Failed to open webcam.")
    exit()

kalman_x = SimpleKalman()
kalman_y = SimpleKalman()

servo_x, servo_y = 90, 80  # Tilt starts at 80
dead_zone = 30             # Pixel dead zone
max_angle_step = 2         # Max servo step per update

ser.write(b"90,80,0\n")
print("Initial servo position set: Pan=90°, Tilt=80°, Laser OFF")

def clamp(val, minv, maxv):
    return max(minv, min(maxv, val))

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.resize(frame, (640, int(frame.shape[0] * 640 / frame.shape[1])))

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cl = clahe.apply(l)
    enhanced = cv2.merge((cl, a, b))
    frame = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)

    height, width = frame.shape[:2]
    center_x, center_y = width // 2, height // 2

    results = model.predict(source=frame, conf=0.5, verbose=False)
    detected = False

    if results and results[0].boxes:
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            w, h = x2 - x1, y2 - y1
            area = w * h

            # Skip overly large detections
            if area > 0.6 * width * height:
                continue

            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            kx = int(kalman_x.update(cx))
            ky = int(kalman_y.update(cy))

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (kx, ky), 5, (255, 0, 0), -1)

            offset_x = kx - center_x
            offset_y = ky - center_y

            if abs(offset_x) > dead_zone or abs(offset_y) > dead_zone:
                delta_x = clamp((offset_x / (width // 2)) * 10, -max_angle_step, max_angle_step)
                delta_y = clamp((offset_y / (height // 2)) * 10, -max_angle_step, max_angle_step)

                servo_x = clamp(servo_x + int(delta_x), 80, 100)
                servo_y = clamp(servo_y + int(delta_y), 60, 95)

                ser.write(f"{servo_x},{servo_y},1\n".encode())
                print(f"📡 Tracking - Pan: {servo_x}°, Tilt: {servo_y}°, Laser ON")
            detected = True
            break

    if not detected:
        if servo_x != 90 or servo_y != 80:
            servo_x = int(servo_x + np.sign(90 - servo_x))
            servo_y = int(servo_y + np.sign(80 - servo_y))
            ser.write(f"{servo_x},{servo_y},0\n".encode())
            print("No detection - Resetting Pan/Tilt...")

    cv2.imshow("Drone Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.write(b"90,80,0\n") 
ser.close()
print("Serial port closed cleanly")
