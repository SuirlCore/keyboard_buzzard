#program to listen to gpio switch and send the shutdown command when no power at port

import RPi.GPIO as GPIO
import time
import os

# Settings
PIN_TO_MONITOR = 17  # Change this to the GPIO pin you are using

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_TO_MONITOR, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Assume pull-up

def shutdown():
    print("Pin went LOW! Shutting down...")
    os.system("sudo shutdown now")

try:
    print(f"Monitoring GPIO {PIN_TO_MONITOR} for LOW signal...")
    while True:
        if GPIO.input(PIN_TO_MONITOR) == GPIO.LOW:
            shutdown()
            break
        time.sleep(0.1)  # Small delay to avoid CPU overuse
except KeyboardInterrupt:
    print("Program interrupted")
finally:
    GPIO.cleanup()