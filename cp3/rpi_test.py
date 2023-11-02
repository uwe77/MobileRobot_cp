#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
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
# =============================================================================
# motor variables===============================================================
left_speed = None
right_speed = None

count = 0
mid_speed = 50
diff_speed = 10
# =============================================================================
# state variables===============================================================
state = [0,1,2,3,4]
state_index = 0
last_state = 2

# =============================================================================
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(left_pin, GPIO.IN)
    GPIO.setup(right_pin, GPIO.IN)
    GPIO.setup(mid_pin, GPIO.IN)
    left_speed = 0
    right_speed = 0
    count = 0
    last_state = 1
    state_index = 0

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
    global sw_mid, sw_right, sw_left, light_value
    rospy.init_node('center_manager')
    rospy.Subscriber('lm393_data', Int32, read_info)
    pub_L = rospy.Publisher('motor_v_left', Int32, queue_size=1)
    pub_R = rospy.Publisher('motor_v_right', Int32, queue_size=1)
    setup()

    GPIO.setmode(GPIO.BCM)
    while not rospy.is_shutdown():
        try:
            if state[state_index] == 0: # all stop
                last_state = 0
                set_motorLnR_v(0,0)
                if sw_mid == GPIO.LOW:
                    state_index = 1
                    last_light_value = 1000

            elif state[state_index] == 1: # left speed faster
                set_motorLnR_v(mid_speed+diff_speed,mid_speed)
                last_state = 1
                last_light_value = light_value
                if sw_left == GPIO.HIGH and sw_right == GPIO.HIGH:
                    state_index = 4
                elif sw_left == GPIO.HIGH or sw_right == GPIO.HIGH:
                    state_index = 3
                if light_value > last_light_value + light_tol:
                    last_light_value = 1000
                    state_index = 2

            elif state[state_index] == 2: # right speed faster
                set_motorLnR_v(mid_speed,mid_speed+diff_speed)
                last_state = 2
                last_light_value = light_value
                if sw_left == GPIO.HIGH and sw_right == GPIO.HIGH:
                    state_index = 4
                elif sw_left == GPIO.HIGH or sw_right == GPIO.HIGH:
                    state_index = 3
                if light_value > last_light_value + light_tol:
                    last_light_value = 1000
                    state_index = 1

            elif state[state_index] == 3: # single side sw on, go back
                if count == 0:
                    count += 1
                    set_motorLnR_v(-mid_speed + diff_speed,-mid_speed +diff_speed)
                elif count >= 10000:
                    count = 0
                    last_light_value = 1000
                    if last_state == 1:
                        state_index = 2
                        last_state = 3
                    elif last_state == 2:
                        state_index = 1
                        last_state = 3
                elif count < 10000:
                    count += 1
                    
            elif state[state_index] == 4: # both side sw on go back then state5
                if count == 0:
                    count += 1
                    set_motorLnR_v(-mid_speed + diff_speed,-mid_speed +diff_speed)
                elif count >= 10000:
                    count = 0
                    state_index = 5
                    last_state = 4
                elif count < 10000:
                    count += 1

            elif state[state_index] == 5: # turn around then state_index = last_state
                if count == 0:
                    count += 1
                    set_motorLnR_v(-mid_speed + diff_speed,mid_speed - diff_speed)
                elif count >= 10000:
                    count = 0
                    last_light_value = 1000
                    state_index = last_state
                    last_state = 5
                elif count < 10000:
                    count += 1
            
            if sw_mid == GPIO.HIGH:
                state_index = 0
            # print "right_v: ", right_speed
            # print "left_v:  ", left_speed
            pub_L.publish(left_speed)
            pub_R.publish(right_speed)
            rospy.sleep(0.5)
        except ValueError:
            pass

if __name__ == '__main__':
    main()