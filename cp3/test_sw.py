#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins you want to read from
left_pin = 17
right_pin = 18
mid_pin = 27
# Set up the GPIO pins as inputs
GPIO.setup(left_pin, GPIO.IN)
GPIO.setup(right_pin, GPIO.IN)
GPIO.setup(mid_pin, GPIO.IN)

try:
    while True:
        # Read the state of the left GPIO pin
        input_state1 = GPIO.input(left_pin)

        # Read the state of the right GPIO pin
        input_state2 = GPIO.input(right_pin)

        #Read the state of the mid GPIO pin
        input_state3 = GPIO.input(mid_pin)

        print("GPIO pin {} is {}".format(left_pin, "HIGH" if input_state1 == GPIO.HIGH else "LOW"))
        print("GPIO pin {} is {}".format(right_pin, "HIGH" if input_state2 == GPIO.HIGH else "LOW"))
        print("GPIO pin {} is {}".format(mid_pin, "HIGH" if input_state3 == GPIO.HIGH else "LOW"))

except KeyboardInterrupt:
    print("Program terminated by user")
finally:
    # Clean up and reset GPIO settings
    GPIO.cleanup()