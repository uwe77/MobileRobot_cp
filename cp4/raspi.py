#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float32
import RPi.GPIO as GPIO
# sensor variables==============================================================
light_value = 0
last_light_value = 1000
light_tol = 20
target_light = 0
min_light = 1000
left_pin = 17
right_pin = 18
mid_pin = 27
sw_mid = None
sw_right = None
sw_left = None
ir_data = 0.
gate1_ratio = 0.2
gate2_ratio = 0.4
# Set up the GPIO pins as inputs
# =============================================================================
# motor variables===============================================================
left_speed = 0
right_speed = 0
count = 0
mid_speed = 80
diff_speed = 40
# =============================================================================
# state variables===============================================================
state = [0,1,2,3,4,5]
state_index = 0
stage_2 = False
# =============================================================================
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(left_pin, GPIO.IN)
    GPIO.setup(right_pin, GPIO.IN)
    GPIO.setup(mid_pin, GPIO.IN)
    left_speed = 0
    right_speed = 0
    count = 0
    state_index = 0
    min_light = 1000
def set_motorLnR_v(l, r):
    left_speed = l
    right_speed = r
def lm393_info_reader(msg):
    global light_value
    light_value = msg.data
def ir_ratio_reader(msg):
    global ir_data, sw_mid, sw_right, sw_left
    ir_data = msg.data
    sw_mid = GPIO.input(mid_pin)
    sw_right = GPIO.input(right_pin)
    sw_left = GPIO.input(left_pin)
def main():
    global sw_mid, sw_right, sw_left, light_value, min_light, last_light_value, target_light, state_index, count, left_speed, right_speed, ir_data, stage_2
    rospy.init_node('center_manager')
    rospy.Subscriber('lm393_data', Int32, lm393_info_reader)
    rospy.Subscriber('ir_ratio', Float32, ir_ratio_reader)
    pub_L = rospy.Publisher('motor_v_left', Int32, queue_size=1)
    pub_R = rospy.Publisher('motor_v_right', Int32, queue_size=1)
    setup()
    GPIO.setmode(GPIO.BCM)
    while not rospy.is_shutdown():
        try:
            if state[state_index] == 0: # all stop
                last_state = 0
                left_speed = 0
                right_speed = 0
                if sw_mid == GPIO.LOW:
                    state_index = 1
                    last_light_value = 1000
                    target_light = 900
                    min_light = 1000
                    
            elif state[state_index] == 1: # foward
                left_speed = mid_speed
                right_speed = mid_speed
                if min_light > light_value:
                    min_light = light_value
                if target_light+light_tol < light_value and min_light+light_tol > light_value:
                    state_index = 3
                    last_light_value = 1000
            elif state[state_index] == 2: # single sw on, go back
                if count == 0:
                    left_speed = -mid_speed + diff_speed
                    right_speed = -mid_speed + diff_speed
                    count += 1
                elif count >= 10:
                    count = 0
                    last_light_value = 1000
                    state_index = 3
                elif count < 10:
                    count += 1
            elif state[state_index] == 3: # turn around finding the lightest source
                left_speed = -mid_speed+diff_speed
                right_speed = mid_speed-diff_speed
                count += 1
                if last_light_value+light_tol < light_value or light_value < min_light or count >= 70:
                    state_index = 4
                    last_light_value = 1000
                    count = 0
                if min_light > last_light_value:
                    min_light = last_light_value
                elif min_light > light_value:
                    min_light = light_value
                last_light_value = light_value
            elif state[state_index] == 4:
                if count == 0:
                    count += 1
                    left_speed = mid_speed-diff_speed
                    right_speed = -mid_speed+diff_speed
                elif count >= 2:
                    count = 0
                    state_index = 1
                    left_speed = mid_speed
                    right_speed = mid_speed
                    target_light = light_value
                elif count < 2:
                    count += 1
            elif state[state_index] == 5:
                pass
#==============================================================================
            if not stage_2:
                if sw_mid == GPIO.HIGH:
                    stage_2 = True
                    state_index = 5
                else:
                    if sw_left == GPIO.HIGH or sw_right == GPIO.HIGH:
                        state_index = 2
                        count = 0
                        last_light_value = 1000
            else:
                if sw_mid == GPIO.LOW:
                    stage_2 = False
                    state_index = 0

            pub_L.publish(left_speed)
            pub_R.publish(right_speed)
            rospy.sleep(0.1)
        except ValueError:
            pass
if __name__ == '__main__':
    main()