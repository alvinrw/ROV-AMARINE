import time
from pymavlink import mavutil

# Initialize the MAVLink connection
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600)  # Update with your connection information
master.wait_heartbeat()



master.mav.set_mode_send(
master.target_system,
mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
mavutil.mavlink.MAV_MODE_MANUAL_DISARMED
)
time.sleep(1)

master.mav.command_long_send(
master.target_system,
master.target_component,
mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
0,
1, 0, 0, 0, 0, 0, 0)

# Define the function to send manual control commands
def send_manual_control(roll, pitch, yaw, throttle):
    master.mav.manual_control_send(
        master.target_system,
        roll,
        pitch,
        yaw,
        throttle,
        0
    )

# Define the function to move forward
def move_forward(distance):
    duration = int(distance / 0.1)  # Adjust the duration based on the desired distance and ROV speed
    start_time = time.time()


    while time.time() - start_time < duration:
        send_manual_control(1900, 400, 500, 500)  # Move forward with full pitch
        time.sleep(0.1)
        print("Maju")

    send_manual_control(500, 500, 500, 500)  # Stop movement

# Define the function to move down
def move_down(distance):
    duration = int(distance / 0.1)  # Adjust the duration based on the desired distance and ROV speed
    start_time = time.time()
    while time.time() - start_time < duration:
        send_manual_control(500, 500, -1000, 500)  # Move down with full throttle down
        time.sleep(0.1)
        print("kebawah")

    send_manual_control(500, 500, 500, 500)  # Stop movement

# Define the function to turn left
def turn_left(angle):
    duration = int(angle / 10)  # Adjust the duration based on the desired angle and ROV's turning speed
    start_time = time.time()
    while time.time() - start_time < duration:
        send_manual_control(-1000, 500, -1000, 500)  # Rotate left with roll and yaw
        time.sleep(0.1)
        print("belok kiri")

    send_manual_control(500, 500, 500, 500)  # Stop movement

# Define the function to turn right
def turn_right(angle):
    duration = int(angle / 10)  # Adjust the duration based on the desired angle and ROV's turning speed
    start_time = time.time()
    while time.time() - start_time < duration:
        send_manual_control(1000, 500, 1000, 500)  # Rotate right with roll and yaw
        time.sleep(0.1)
        print("belok kanan")

    send_manual_control(500, 500, 500, 500)  # Stop movement

# Define the function to move backward
def move_backward(distance):
    duration = int(distance / 0.1)  # Adjust the duration based on the desired distance and ROV speed
    start_time = time.time()
    while time.time() - start_time < duration:
        send_manual_control(-1000, 500, 500, 500)  # Move backward with full negative pitch
        time.sleep(0.1)
        print("mundur")

    send_manual_control(500, 500, 500, 500)  # Stop movement

# Example usage
# move_x(delay)

move_down(1)

move_forward(2)  

turn_left(90)  

turn_right(90) 

move_backward(1)  

#disarm command
master.mav.command_long_send(
master.target_system,
master.target_component,
mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
0,
0, 0, 0, 0, 0, 0, 0)