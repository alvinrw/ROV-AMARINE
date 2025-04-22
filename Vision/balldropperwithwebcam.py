from ultralytics import YOLO
import cv2
import time 
import math

# Load the YOLOv8 model
model = YOLO("weights/best.pt")  # Replace with your trained model path

# Define colors and fonts for drawing
RETICLE_COLOR = (0, 255, 0)  # Green color for the reticle
TEXT_COLOR = (0, 0, 255)  # Red color for the text
BBOX_COLOR = (0, 255, 0)  # Green color for the bounding box
FONT = cv2.FONT_HERSHEY_SIMPLEX

def draw_reticle(frame, center_x, center_y, radius):
    """
    Draws a reticle with a plus sign at the center of the frame.
    """
    cv2.line(frame, (center_x - radius, center_y), (center_x + radius, center_y), RETICLE_COLOR, 2)
    cv2.line(frame, (center_x, center_y - radius), (center_x, center_y + radius), RETICLE_COLOR, 2)

def is_near_center(bbox, center_x, center_y, threshold):
    """
    Checks if the bounding box is near the center of the frame within a given threshold.
    """
    bbox_center_x = (bbox[0] + bbox[2]) / 2
    bbox_center_y = (bbox[1] + bbox[3]) / 2
    distance = math.sqrt((bbox_center_x - center_x)**2 + (bbox_center_y - center_y)**2)
    return distance < threshold

# Open the video file
#video_path = "topViewBucket.mp4"  # Replace with the path to your video file
cap = cv2.VideoCapture(0) # with webcam camera index 0

while True:
    # Capture frame from the video
    ret, frame = cap.read()

    # Break the loop if there are no more frames
    if not ret:
        break

    # Get the frame dimensions
    height, width, _ = frame.shape

    # Define the center and radius of the reticle
    center_x = width // 2
    center_y = height // 2
    radius = min(width, height) // 4  # Adjust the radius as needed

    # Draw the reticle
    draw_reticle(frame, center_x, center_y, radius)

    # Detect objects using YOLOv8
    results = model(frame, stream=True)

    # Check for the "bucket" class
    for result in results:
        boxes = result.boxes
        for box in boxes:
            class_id = box.cls
            if model.names[int(class_id)] == "bucket":
                # Draw the bounding box
                bbox = box.xyxy[0].cpu().numpy().astype(int)
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), BBOX_COLOR, 2)

                # Check if the bounding box is near the center of the reticle
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

    # Show the frame
    cv2.imshow("YOLOv8 Detection", frame)

    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()
