#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
from RPi.GPIO import HIGH as HIGH, LOW as LOW
# sensor variables==============================================================
light_value = 0
last_light_value = 1000
light_tol = 10

left_pin = 17
right_pin = 18
mid_pin = 27
sw_mid = None
sw_right = None
sw_left = None
# Set up the GPIO pins as inputs
GPIO.setmode(GPIO.BCM)
GPIO.setup(left_pin, GPIO.IN)
GPIO.setup(right_pin, GPIO.IN)
GPIO.setup(mid_pin, GPIO.IN)
# =============================================================================
# motor variables===============================================================
left_speed = None
right_speed = None

count = 0
mid_speed = 50
diff_speed = 10
# =============================================================================
# state variables===============================================================
state = 0
last_state = 2
back_situation = 0

# =============================================================================

def set_motorLnR_v(l, r):
    left_speed = l
    right_speed = r

def read_info(msg):
    global sw_mid, sw_right, sw_left, light_value
    light_value = msg.data
    sw_mid = GPIO.input(mid_pin)
    sw_right = GPIO.input(right_pin)
    sw_left = GPIO.input(left_pin)

def main():
    global light_value
    rospy.init_node('center_manager')
    rospy.Subscriber('lm393_data', Int32, read_info)
    pub_L = rospy.Publisher('motor_v_left', Int32, queue_size=1)
    pub_R = rospy.Publisher('motor_v_right', Int32, queue_size=1)
    state = 0
    last_state = 1
    back_situation = 0

    GPIO.setmode(GPIO.BCM)
    while not rospy.is_shutdown():
        try:
            if state == 0: # all stop
                if state != last_state:
                    last_state = state
                    set_motorLnR_v(0,0)
                    last_light_value = 1000
                else:
                    if sw_mid == LOW:
                        state = 1
            print "right_v: ", right_speed
            print "left_v:  ", left_speed
            pub_L.publish(left_speed)
            pub_R.publish(right_speed)
            rospy.sleep(0.5)
        except ValueError:
            pass
        # finally:
        #     # Clean up and reset GPIO settings
        #     GPIO.cleanup()
if __name__ == '__main__':
    main()