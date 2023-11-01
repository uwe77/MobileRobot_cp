#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


res = None

def input_callback(msg):
    global res
    res = msg.data

def main():
    global res
    rospy.init_node('lm393_reciver')
    # pub = rospy.Publisher('keyboard_inputnum', Int32, queue_size=1)
    rospy.Subscriber('lm393_data', Int32, input_callback)

    while not rospy.is_shutdown():
        try:
            if res is not None:
                print "message from lm393 is ",res
            rospy.sleep(0.5)
        except ValueError:
            pass
        

if __name__ == '__main__':
    main()

# import RPi.GPIO as GPIO

# # Set the GPIO mode to BCM
# GPIO.setmode(GPIO.BCM)

# # Define the GPIO pins you want to read from
# lm393_pin = 23
# # Set up the GPIO pins as inputs
# GPIO.setup(lm393_pin, GPIO.IN)


# try:
#     while True:
#         # Read the state of the left GPIO pin
#         input_state1 = GPIO.input(lm393_pin)

#         print("GPIO pin {} is {}".format(lm393_pin, "HIGH" if input_state1 == GPIO.HIGH else "LOW"))

# except KeyboardInterrupt:
#     print("Program terminated by user")
# finally:
#     # Clean up and reset GPIO settings
#     GPIO.cleanup()