import RPi.GPIO as GPIO
import time

# Pin Configuration
STEP_PIN = 17  
DIR_PIN = 27    
MODE_PINS = (14, 15, 18)  # Microstepping mode pins (if applicable)

# GPIO Setup
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)


# Function to use open and close stepper moto
def step(direction):
    GPIO.output(DIR_PIN, direction)  # Set direction
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(0.001)  # Adjust delay if needed
    GPIO.output(STEP_PIN, GPIO.LOW)

# Example usage: Rotate motor clockwise for 100 steps
GPIO.output(DIR_PIN, GPIO.HIGH)  # Clockwise direction
for _ in range(100):
    step(GPIO.HIGH)

# Clean up
GPIO.cleanup()