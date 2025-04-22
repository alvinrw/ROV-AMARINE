import cv2
import time
import math
from ultralytics import YOLO
import numpy as np

# Load the YOLOv8 model
model = YOLO("weights/best.pt")# Replace with your trained model path

# Define colors and fonts for drawing
RETICLE_COLOR = (0, 255, 0)
TEXT_COLOR = (0, 0, 255)
BBOX_COLOR = (0, 255, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX

# Helper functions
def draw_reticle(frame, center_x, center_y, radius):
    cv2.line(frame, (center_x - radius, center_y), (center_x + radius, center_y), RETICLE_COLOR, 2)
    cv2.line(frame, (center_x, center_y - radius), (center_x, center_y + radius), RETICLE_COLOR, 2)

def is_near_center(bbox, center_x, center_y, threshold):
    bbox_center_x = (bbox[0] + bbox[2]) / 2
    bbox_center_y = (bbox[1] + bbox[3]) / 2
    distance = math.sqrt((bbox_center_x - center_x)**2 + (bbox_center_y - center_y)**2)
    return distance < threshold

def apply_water_filter(image):
    b, g, r = cv2.split(image)  
    b = cv2.addWeighted(b, 1.1, np.zeros(b.shape, b.dtype), 0, 0)  # Increase blue slightly
    g = cv2.addWeighted(g, 1.05, np.zeros(g.shape, g.dtype), 0, 0)  # Increase green slightly
    return cv2.merge([b, g, r])

# Open the video file
video_path = "video/topViewBucket.mp4" # Replace with the path to your video file
cap = cv2.VideoCapture(video_path)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    height, width, _ = frame.shape
    center_x = width // 2
    center_y = height // 2
    radius = min(width, height) // 4

    draw_reticle(frame, center_x, center_y, radius)

    results = model(frame, stream=True)

    for result in results:
        boxes = result.boxes
        for box in boxes:
            class_id = box.cls
            if model.names[int(class_id)] == "bucket":
                bbox = box.xyxy[0].cpu().numpy().astype(int)
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), BBOX_COLOR, 2)

                # Apply basic water filter to the detected bucket
                bucket_region = frame[bbox[1]:bbox[3], bbox[0]:bbox[2]]
                filtered_bucket_region = apply_water_filter(bucket_region)
                frame[bbox[1]:bbox[3], bbox[0]:bbox[2]] = filtered_bucket_region  

                if is_near_center(bbox, center_x, center_y, radius // 2):
                    text = "ball dropped"
                    text_size, _ = cv2.getTextSize(text, FONT, 1, 2)
                    text_x = center_x - text_size[0] // 2
                    text_y = center_y + text_size[1] // 2
                    cv2.putText(frame, text, (text_x, text_y), FONT, 1, TEXT_COLOR, 2)

    # Calculate and display FPS 
    start_time = time.time()
    end_time = time.time()
    fps = 1 / (end_time - start_time)
    cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), FONT, 0.75, TEXT_COLOR, 2)

    cv2.imshow("YOLOv8 Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()