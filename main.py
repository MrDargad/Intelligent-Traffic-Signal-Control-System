import cv2
import time
import threading
from ultralytics import YOLO
import torch
import serial
import numpy as np

# Set threading for better performance
torch.set_num_threads(1)

# Load YOLO model (use 'yolov8n.pt' for best speed)
model = YOLO('yolov8n.pt')

# IP camera URLs (replace with your IPs)
camera_urls = [
    "http://192.168.1.50:8080/video",  # Mobile 1
    "http://192.168.1.51:8080/video",  # Mobile 2
    "http://192.168.1.35:8080/video",  # Mobile 3
]

# Vehicle classes (bicycle, car, motorcycle, bus, truck) from COCO dataset
vehicle_classes = {1, 2, 3, 5}

# Emergency vehicle class (e.g., ambulance, fire truck)
emergency_classes = {7, 10}  # Update with actual class index if needed

# Frame skipping factor (higher = faster, less accurate)
FRAME_SKIP = 20

# Constants
MIN_GREEN_TIME = 10
MAX_GREEN_TIME = 40
YELLOW_TIME = 5

# Global variables
vehicle_counts = [0] * len(camera_urls)
emergency_detected = [False] * len(camera_urls)
camera_active = [True] * len(camera_urls)
green_light_status = [False] * len(camera_urls)
active_green_time = [0] * len(camera_urls)
allocated_green_time = [MIN_GREEN_TIME] * len(camera_urls)
total_green_time = [0] * len(camera_urls)
extension_occurred = [False] * len(camera_urls)  # Track if extension has occurred

# Initialize serial communication with Arduino
arduino = serial.Serial('COM14', 9600, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

def send_signal_to_arduino(signal):
    arduino.write((signal + '\n').encode())
    print(f"Sent signal to Arduino: {signal}")

# Process each camera feed in a separate thread
def process_camera(index, url):
    global vehicle_counts, emergency_detected, camera_active

    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print(f"Error: Cannot open camera {index + 1}")
        vehicle_counts[index] = 0
        emergency_detected[index] = False
        camera_active[index] = False
        return

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"Failed to capture from camera {index + 1}")
            vehicle_counts[index] = 0
            emergency_detected[index] = False
            camera_active[index] = False
            break

        frame_count += 1
        if frame_count % FRAME_SKIP != 0:
            continue

        frame_resized = cv2.resize(frame, (320, 240))

        results = model(frame_resized, verbose=False)

        vehicle_count = 0
        emergency_present = False

        for box in results[0].boxes:
            cls = int(box.cls[0])
            if cls in vehicle_classes:
                vehicle_count += 1
                # Draw rectangle for vehicle class
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if cls in emergency_classes:
                emergency_present = True
                # Draw rectangle for emergency class
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 0, 255), 2)

        vehicle_counts[index] = vehicle_count
        emergency_detected[index] = emergency_present
        camera_active[index] = True

        # Create a black image for the side panel
        side_panel = np.zeros((240, 320, 3), dtype=np.uint8)

        # Add text information to the side panel
        cv2.putText(side_panel, f"Vehicles: {vehicle_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        status_text = "Emergency: Yes" if emergency_present else "Emergency: No"
        cv2.putText(side_panel, status_text, (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(side_panel, f"Green Light: {'Active' if green_light_status[index] else 'Inactive'}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(side_panel, f"Active Green Time: {active_green_time[index]}s", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(side_panel, f"Allocated Green Time: {allocated_green_time[index]}s", (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        cv2.putText(side_panel, f"Total Green Time: {total_green_time[index]}s", (10, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Concatenate the annotated frame and the side panel
        combined_frame = np.hstack((frame_resized, side_panel))

        # Display the combined frame
        cv2.imshow(f"Camera {index + 1}", combined_frame)

        # Exit when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Traffic light control function
def control_traffic_lights():
    global vehicle_counts, emergency_detected, green_light_status, active_green_time, allocated_green_time, total_green_time, extension_occurred
    TCT = (len(camera_urls) * MIN_GREEN_TIME) + ((len(camera_urls) - 1) * YELLOW_TIME)
    last_normal_tct = TCT

    while True:
        total_vehicle_count = sum(vehicle_counts)
        if total_vehicle_count == 0:
            green_times = [MIN_GREEN_TIME] * len(camera_urls)
        else:
            green_times = [
                max(MIN_GREEN_TIME, min(MAX_GREEN_TIME, (vehicle_count / total_vehicle_count) * TCT))
                for vehicle_count in vehicle_counts
            ]

        for i in range(len(camera_urls)):
            if emergency_detected[i]:
                print(f"Emergency detected on Camera {i + 1}. Allocating maximum green time.")
                green_times[i] = MAX_GREEN_TIME
                for j in range(len(camera_urls)):
                    if j != i:
                        green_times[j] = MIN_GREEN_TIME

        for i in range(len(camera_urls)):
            allocated_green_time[i] = green_times[i]
            extension_occurred[i] = False  # Reset extension flag for new cycle
            print(f"Green light for Camera {i + 1} for {green_times[i]} seconds")
            send_signal_to_arduino(f"G{i+1}")  # Send green light signal to Arduino
            start_time = time.time()
            active_green_time[i] = 0
            green_light_status[i] = True
            while active_green_time[i] < green_times[i]:
                if emergency_detected[i]:
                    print(f"Emergency on Camera {i + 1}. Extending green time.")
                    green_times[i] = MAX_GREEN_TIME
                    emergency_detected[i] = False  # Reset emergency flag after handling
                else:
                    # Check if any other signal has an emergency vehicle detected
                    for j in range(len(camera_urls)):
                        if j != i and emergency_detected[j] and active_green_time[i] >= MIN_GREEN_TIME:
                            print(f"Emergency detected on Camera {j + 1}. Stopping green light for Camera {i + 1}.")
                            green_times[i] = active_green_time[i]  # Stop the current green light
                            break
                time.sleep(1)
                active_green_time[i] += 1
                print(f"Green light for Camera {i + 1} active for {active_green_time[i]} seconds")
                if active_green_time[i] >= MIN_GREEN_TIME and vehicle_counts[i] == 0:
                    print(f"No vehicles detected on Camera {i + 1}. Transitioning to yellow light.")
                    break
                if active_green_time[i] >= green_times[i] - 2 and not extension_occurred[i]:
                    # Check for additional vehicles and extend green time if needed
                    new_vehicle_count = vehicle_counts[i]
                    total_vehicle_count = sum(vehicle_counts)
                    if new_vehicle_count > 0:
                        alpha = new_vehicle_count / total_vehicle_count
                        additional_time = alpha * (new_vehicle_count)
                        if green_times[i] + additional_time > MAX_GREEN_TIME:
                            additional_time = MAX_GREEN_TIME - green_times[i]
                        green_times[i] += additional_time
                        allocated_green_time[i] += additional_time  # Update allocated green time
                        extension_occurred[i] = True  # Mark extension as occurred
                        print(f"Extending green time for Camera {i + 1} by {additional_time} seconds")

            total_green_time[i] = active_green_time[i]
            green_light_status[i] = False
            send_signal_to_arduino(f"Y{i+1}")  # Send yellow light signal to Arduino
            print(f"Yellow light for {YELLOW_TIME} seconds")
            time.sleep(YELLOW_TIME)
            send_signal_to_arduino(f"R{i+1}")  # Send red light signal to Arduino

        if not any(emergency_detected):
            last_normal_tct = sum(green_times) + ((len(camera_urls) - 1) * YELLOW_TIME)
        TCT = last_normal_tct

# Launch threads for each camera
threads = [threading.Thread(target=process_camera, args=(i, url)) for i, url in enumerate(camera_urls)]

# Start all threads
for thread in threads:
    thread.start()
    time.sleep(20)  # Wait for 20 seconds for mobile to connect

# Start traffic light control
traffic_light_thread = threading.Thread(target=control_traffic_lights)
traffic_light_thread.start()

# Wait for all threads to finish
for thread in threads:
    thread.join()
traffic_light_thread.join()