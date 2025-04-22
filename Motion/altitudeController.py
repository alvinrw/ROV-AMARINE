import time
from pymavlink import mavutil
from brping import Ping1D

myPing = Ping1D()
myPing.connect_serial("/dev/ttyUSB0", 115200)
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)

# Wait for the first heartbeat to make sure the connection has been established
master.wait_heartbeat()

DESIRED_ALTITUDE = 2  # Meters
CHANNEL_ASCEND_DESCEND = 3
PWM_NEUTRAL = 1500

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


def calculate_altitude_control(current_distance):
    altitude_error = current_distance - DESIRED_ALTITUDE
    kP = 0.5  # Proportional gain - adjust this value

    correction = kP * altitude_error
    output_pwm = PWM_NEUTRAL + correction

    # Limit output PWM to valid range
    output_pwm = max(1100, min(output_pwm, 1900)) 

    return output_pwm 

# Main loop

while True:
    data = myPing.get_distance()
    if data:
        distance = data["distance"] / 1000  # Convert to meters
        pwm_value = calculate_altitude_control(distance)
        set_rc_channel_pwm(CHANNEL_ASCEND_DESCEND, pwm_value)
    else:
        print("Failed to get distance data")

    time.sleep(0.1)  # Adjust loop rate as needed