#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
from RPi.GPIO import HIGH as HIGH, LOW as LOW
# sensor variables==============================================================
light_value = 0
last_light_value = 0
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
mid_speed = 100
diff_speed = 10
# =============================================================================
# state variables===============================================================
state = 0
last_state = 2
back_situation = 0
# =============================================================================

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

    while not rospy.is_shutdown():
        try:
            if state == 0: # all stop
                if state != last_state:
                    last_state = state
                    left_speed = 0
                    right_speed = 0
                    last_light_value = 0
                    pub_L.publish(left_speed)
                    pub_R.publish(right_speed)
                else:
                    if sw_mid == LOW:
                        state = 2
            elif state == 1: # go back function dont change last_state
                left_speed = -mid_speed
                right_speed = -mid_speed
                last_light_value = 0
                pub_L.publish(left_speed)
                pub_R.publish(right_speed)
                count += 1
                if count >= 100:
                    count = 0
                    if back_situation == 1:
                        state = 3
                        last_state = 1
                    elif back_situation == 2:
                        state = 2
                        last_state = 1
                    elif back_situation == 3:
                        state = 4
            elif state == 2: # forward but left is faster
                if last_state != state:
                    last_state = state
                    left_speed = mid_speed + diff_speed
                    right_speed = mid_speed
                    pub_L.publish(left_speed)
                    pub_R.publish(right_speed)
                else:
                    if sw_left == HIGH and sw_right == HIGH:
                        state = 1
                        back_situation = 3
                    elif sw_left == HIGH:
                        back_situation = 2
                        state = 1
                    elif sw_right == HIGH:
                        back_situation = 1
                        state = 1
                if light_value < last_light_value:
                    state = 3
                last_light_value = light_value
            elif state == 3:
                if last_state != state:
                    last_state = state
                    left_speed = mid_speed
                    right_speed = mid_speed + diff_speed
                    pub_L.publish(left_speed)
                    pub_R.publish(right_speed)
                else:
                    if sw_left == HIGH and sw_right == HIGH:
                        state = 1
                        back_situation = 3
                    elif sw_left == HIGH:
                        back_situation = 2
                        state = 1
                    elif sw_right == HIGH:
                        back_situation = 1
                        state = 1
                if light_value < last_light_value:
                    state = 2
                last_light_value = light_value
            elif state == 4: # trun 180 then state = last_state
                last_state = state
                left_speed = mid_speed
                right_speed = -mid_speed
                pub_L.publish(left_speed)
                pub_R.publish(right_speed)
                count += 1
                if count >= 100:
                    count = 0
                    state = last_state
                    last_state = 4
            if sw_mid == HIGH:
                state = 0
            rospy.sleep(0.5)
        except ValueError:
            pass

if __name__ == '__main__':
    main()