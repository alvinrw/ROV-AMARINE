#Add this to the main startup from the operating system

import serial
import subprocess
import time

# Serial Port Configuration
bluetooth_port = '/dev/rfcomm0'  # Adjust if needed
bluetooth_baud_rate = 9600

# Script Directories (Update these!)
main_a_dir = '/path/to/main_a_script' 
main_b_dir = '/path/to/main_b_script'
thruster_dir = '/path/to/main_b_script'
gripper_dir = '/path/to/main_b_script'
force_stop_dir = '/path/to/main_b_script'
# ... similar for other directories

# Connect to Bluetooth
try:
    ser = serial.Serial(bluetooth_port, bluetooth_baud_rate)
    print("Bluetooth connection established")
except Exception as e:
    print("Error connecting to Bluetooth module:", e)
    exit()

# Main loop
while True:
    if ser.in_waiting > 0:  
        data = ser.readline().decode('utf-8').rstrip()

        if data == "MAIN-A":
            subprocess.Popen(['python', main_a_dir + '/script.py'])
        elif data == "MAIN-B":
            subprocess.Popen(['python', main_b_dir + '/script.py'])
        elif data == "THRUSTER-CHECK":
            subprocess.Popen(['python', thruster_dir + '/script.py'])  
        elif data == "GRIPPER-CHECK":
            subprocess.Popen(['python', gripper_dir + '/script.py'])  
        elif data == "FORCE-STOP":
            subprocess.Popen(['python', force_stop_dir + '/script.py'])  
        else:
            print("Unrecognized command:", data) 

    time.sleep(0.1)  # Small delay to not overload the CPU
