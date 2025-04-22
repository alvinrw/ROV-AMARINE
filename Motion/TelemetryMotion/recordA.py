from pymavlink import mavutil
import json
import os
import time

file_path = '/home/amarine/ROV/Motion/TelemetryMotion/excecutionAutonomous'

# Create a connection to the Pixhawk
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

# Wait for the first heartbeat to make sure the connection has been established
master.wait_heartbeat()

# Get the current date for generating unique file names
current_date = time.strftime("%Y-%m-%d")

# Determine the file name based on the date
file_name = f"ArenaBtype{len(os.listdir('LintasanA')) + 1}.json"

# A list to store the logged RC values
rc_log = []

try:
    while True:
        # Receive the current message
        msg = master.recv_match()

        # Print the received message for debugging
        print("Received message:", msg)

        # If the message is an RC_CHANNELS message, log the values
        if msg is not None and hasattr(msg, 'get_type') and msg.get_type() == 'RC_CHANNELS':
            rc_log.append(msg.to_dict())
except KeyboardInterrupt:
    # If the script is terminated, write the logged movements to a file in the LintasanB folder
    file_path = os.path.join("LintasanB", file_name)
    with open(file_path, 'w') as f:
        json.dump(rc_log, f)

    print(f"RC data saved to: {file_path}")