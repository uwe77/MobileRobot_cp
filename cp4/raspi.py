#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float32
import RPi.GPIO as GPIO
# sensor variables==============================================================
light_value = 0
light_tol = 50
target_light = 0

left_pin = 17
right_pin = 18
mid_pin = 27

sw_mid = None
sw_right = None
sw_left = None

ir_data = 0.
gate2_ratio = 0.225 # 0.15~0.3
gate1_ratio = 0.45 # 0.4~0.5
gate_ratio  = 0.225
ratio_range = 0.1 # 0.075 & 0.05
# Set up the GPIO pins as inputs
# =============================================================================
# motor variables===============================================================
left_speed = 0
right_speed = 0
count = 0
target_count = 0
diff_count = 0
mid_speed = 80
diff_speed = 40
# =============================================================================
# state variables===============================================================
state = [0,1,2,3,4]
state_index = 0
stage_2 = False
end = False
opening = True
# =============================================================================
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(left_pin, GPIO.IN)
    GPIO.setup(right_pin, GPIO.IN)
    GPIO.setup(mid_pin, GPIO.IN)
    left_speed = 0
    right_speed = 0
    count = 0
    target_count = 0
    diff_count = 0
    state_index = 0
    end = False
    opening = True
    gate_ratio = gate2_ratio
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
    global sw_mid, sw_right, sw_left, light_value, target_light, \
            state_index, count, diff_count, target_count, left_speed, \
            right_speed, ir_data, stage_2, gate_ratio, gate1_ratio, gate2_ratio, ratio_range, end, opening
    rospy.init_node('center_manager')
    rospy.Subscriber('lm393_data', Int32, lm393_info_reader)
    rospy.Subscriber('ir_ratio', Float32, ir_ratio_reader)
    pub_L = rospy.Publisher('motor_v_left', Int32, queue_size=1)
    pub_R = rospy.Publisher('motor_v_right', Int32, queue_size=1)
    setup()
    GPIO.setmode(GPIO.BCM)
    while not rospy.is_shutdown():
        try:
            if not stage_2 and not end: # finding ball
                if sw_mid == GPIO.HIGH: # finded ball
                    stage_2 = True
                    state_index = 3
                    count = 0
                    target_count = 0
                    diff_count = 0
                else: # finding ball
                    if sw_left == GPIO.HIGH or sw_right == GPIO.HIGH: # single sw on
                        state_index = 2
                        count = 0
                        target_count = 0
                        diff_count = 0
                        target_light = 1000
            elif stage_2 and not end: # finding gate
                if sw_left == GPIO.HIGH or sw_right == GPIO.HIGH: # single sw on and inside the gate
                    if abs(ir_data-gate1_ratio) < ratio_range:
                        state_index = 0
                    else:
                        state_index = 2
                        count = 0
                        target_count = 0
                        diff_count = 0
#==============================================================================
            if state[state_index] == 0: # all stop
                right_speed = 0
                left_speed = 0
                end = True
                if sw_mid == GPIO.LOW:
                    state_index = 1
                    target_light = 1000
                    count = 0
                    target_count = 0
                    diff_count = 0
                    stage_2 = False
                    end = False
                    opening = True

            elif state[state_index] == 1: # forward
                if not stage_2: # finding ball
                    left_speed = mid_speed
                    right_speed = mid_speed
                    if (target_light+light_tol < light_value) and not opening:
                        state_index = 3
                        target_light = 1000
                    elif target_light > light_value+light_tol:
                        target_light = light_value
                else: # finding gate
                    left_speed = mid_speed
                    right_speed = mid_speed
                    

            elif state[state_index] == 2: # single sw on, go back
                if not stage_2: # finding ball
                    if count == 0:
                        if opening:
                            opening = False
                        left_speed = -mid_speed + diff_speed
                        right_speed = -mid_speed + diff_speed
                        count += 1
                    elif count >= 10:
                        count = 0
                        target_light = 1000
                        state_index = 3
                    elif count < 10:
                        count += 1
                else: # finding gate
                    if abs(ir_data-gate1_ratio) < ratio_range: # inside the gate
                        state_index = 0
                        count = 0
                    else:
                        count += 1
                        left_speed = -mid_speed + diff_speed
                        right_speed = -mid_speed + diff_speed
                        if count >= 10:
                            count = 0
                            state_index = 3

            elif state[state_index] == 3: # turn around
                if not stage_2: # finding ball
                    left_speed = -mid_speed+diff_speed
                    right_speed = mid_speed-diff_speed
                    count += 1
                    if count < 56: #turning 360 degree
                        if target_light > light_value+light_tol:
                            target_light = light_value
                            target_count = count
                    elif count >= 56: #turned 360 degree
                        if 56 - target_count >= 0:
                            diff_count = 56 - target_count
                            state_index = 4
                            count = 0
                            target_count = 0
                        elif count >= 66:
                            state_index = 1
                            count = 0
                            target_count = 0
                            diff_count = 0
                else: # finding gate
                    left_speed = -mid_speed+diff_speed
                    right_speed = mid_speed-diff_speed
                    if abs(ir_data-gate_ratio) < ratio_range:
                        state_index = 1
                        count = 0
                    elif count >= 66:
                        count = 0
                        state_index = 1
                    else:
                        count += 1

            elif state[state_index] == 4: # turn to the target_count source
                if not stage_2: # finding ball
                    left_speed  =  mid_speed-diff_speed
                    right_speed = -mid_speed+diff_speed
                    count += 1
                    if count >= diff_count:
                        state_index = 1
                        count = 0
                        diff_count = 0
                    else:
                        if target_light > light_value + light_tol:
                            target_light = light_value
                else: # finding gate
                    pass
                    # left_speed  =  mid_speed-diff_speed
                    # right_speed = -mid_speed+diff_speed
                    # count += 1
                    # if count >= diff_count:
                    #     state_index = 1
                    #     count = 0
                    #     diff_count = 0target_count
            print "stage2: ", stage_2, "state: ", state_index
            print "count: ", count, "atrget_count: ", target_count, "diff count", diff_count
            print "ir_data: ", ir_data
            print "lm393: ", light_value, "target_l: ", target_light
            pub_L.publish(left_speed)
            pub_R.publish(right_speed)
            rospy.sleep(0.1)
        except ValueError:
            pass

if __name__ == '__main__':
    main()
