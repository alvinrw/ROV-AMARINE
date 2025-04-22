import multiprocessing
import time
import cv2
from ultralytics import YOLO
import math
from pymavlink import mavutil
from brping import Ping1D
import json
import os


# Connection to Pixhawk
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master.wait_heartbeat()

# Condition If connection to PIXHAWK is Successfull
if master.target_system != 0:
    print("Connection successful!")
else:
    print("Connection failed.")

#Connection to ping
myPing = Ping1D()
myPing.connect_serial("/dev/ttyUSB0", 115200)



# --- Function definitions (same as your provided code) ---

# ... (Your existing function definitions for YOLO object detection, 
#     altitude control, and RC override from JSON) ...

#=======================================TELEMETRY FUNCTIONS=========================================#

def set_rc_channel_pwm(channel_id, pwm=1500):

    """ Set RC channel pwm value
    Args:
        id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1:
        print("Channel does not exist.")
        return

    # We only have 8 channels
    #http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE
    if channel_id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[channel_id - 1] = pwm
        master.mav.rc_channels_override_send(
            master.target_system,                # target_system
            master.target_component,             # target_component
            *rc_channel_values)                  # RC channel list, in microseconds.
        

def calculate_altitude_control(current_distance, desired_altitude, pwm_neutral):
    altitude_error = current_distance - desired_altitude
    kP = 0.5  # Proportional gain - adjust this value

    correction = kP * altitude_error
    output_pwm = pwm_neutral + correction

    # Limit output PWM to valid range
    output_pwm = max(1100, min(output_pwm, 1900)) 

    return output_pwm 


#====================================================================================================#


#======================================COMPUTER VISION FUNCTIONS=====================================#

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

#===================================================================================================#

# --- Multiprocessing implementation ---

def yolo_process(video_source=0):  # Add parameter for video source
   
        # Initialize video capture
        cap = cv2.VideoCapture(video_source) 

        # Load the YOLOv8 model
        model = YOLO("weights/best.pt")

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

            # Calculate and display FPS (corrected)
            start_time = time.time()  # Move start_time before processi
            end_time = time.time()
            fps = 1 / (end_time - start_time)  # Calculate FPS after processing
            cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), FONT, 0.75, TEXT_COLOR, 2)

            # Show the frame
            cv2.imshow("YOLOv8 Detection", frame)

            # Exit if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()  # Release video capture in a finally block
        cv2.destroyAllWindows()  # Close all windows


def altitude_control_process():

        data = myPing.get_distance()
        if data:
            distance = data["distance"] / 1000  # Convert to meters
            # Assuming desired_altitude and pwm_neutral are defined or passed as arguments
            pwm_value = calculate_altitude_control(distance, desired_altitude, pwm_neutral)
            set_rc_channel_pwm(3, pwm_value)
        else:
            print("Failed to get distance data")

        time.sleep(0.1)  # Adjust loop rate as needed

def rc_override_process(json_file_path):
   
        # ... (Your existing RC override code) .
        try:
            with open(json_file_path, 'r') as f:
                rc_log = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            print(f"Error loading RC log: {e}")
            exit()

        # Send RC_CHANNELS_OVERRIDE messages from the JSON data
        for log in rc_log:
            # Extract the PWM values
            chan_raw = [log.get('chan{}_raw'.format(i), 0) for i in range(1, 19)]

            # Send an RC_CHANNELS_OVERRIDE message with the logged values
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component, *chan_raw
            )

            # Wait a short time before sending the next message
            time.sleep(0.24)

        print("Autonomous execution completed.")
   


if __name__ == "__main__":
    # Define parameters for each process\n    yolo_model_path = \    altitude_control_channel = 3

    rc_override_json_path = ""

    # Create processes
   # Assuming the following arguments are needed based on the previous code snippets:
    yolo_model_path = "..."  # Path to the YOLO model
    camera_index = 0  # Camera index
    altitude_sensor_device = "..."  # Altitude sensor device path
    altitude_sensor_baudrate = 115200  # Baud rate for altitude sensor
    desired_altitude = 2  # Desired altitude in meters
    altitude_control_channel = 3  # Channel for altitude control
    pwm_neutral = 1500  # Neutral PWM value
    mavlink_connection_string = "/dev/ttyACM0"  # Mavlink connection string
    mavlink_baudrate = 115200  # Mavlink baud rate

    yolo_processed = multiprocessing.Process(target=yolo_process, args=(yolo_model_path, camera_index))
    altitude_processed = multiprocessing.Process(target=altitude_control_process, args=(altitude_sensor_device, altitude_sensor_baudrate, desired_altitude, altitude_control_channel, pwm_neutral))
    rc_override_processed = multiprocessing.Process(target=rc_override_process, args=(rc_override_json_path, mavlink_connection_string, mavlink_baudrate))

    # Start processes
    yolo_processed.start()
    altitude_processed.start()
    rc_override_processed.start()

    # Wait for processes to finish (optional, depending on your application)
    yolo_processed.join()
    altitude_processed.join()
    rc_override_processed.join()

    print("All processes finished.")