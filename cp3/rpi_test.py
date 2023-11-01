#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int32

light_v = 0
dir = 0.0
sw_mid = False
sw_right = False
sw_left = False
mid_speed = 100
diff_speed = 10
back_dis = 10


left_speed = 0
right_speed = 0
count = 0
state = 0
last_state = 1
back_situation = 0

def read_info():
    sw_mid = GPIO.input(17)
    sw_right = GPIO.input(27)
    sw_left = GPIO.input(22)
    light_v = GPIO.input(18) #temp

def main():
    rospy.init_node('pwm_pub')
    pub_L = rospy.Publisher('motor_v_left', Int32, queue_size=1)
    pub_R = rospy.Publisher('motor_v_right', Int32, queue_size=1)

    while not rospy.is_shutdown():
        try:
            if state == 0: #all stop
                if state != last_state:
                    last_state = state
                    left_speed = 0
                    right_speed = 0
                    pub_L.publish(left_speed)
                    pub_R.publish(right_speed)
                else:
                    if not sw_mid:
                        state = 1
            elif state == 1: # go back function dont change last_state
                left_speed = -mid_speed
                right_speed = -mid_speed
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
            elif state == 2: #forward but left is faster
                if last_state != state:
                    last_state = state
                    left_speed = mid_speed + diff_speed
                    right_speed = mid_speed
                    pub_L.publish(left_speed)
                    pub_R.publish(right_speed)
                else:
                    if sw_left and sw_right:
                        state = 1
                        back_situation = 3
                    elif sw_left:
                        back_situation = 2
                        state = 1
                    elif sw_right:
                        back_situation = 1
                        state = 1
            elif state == 3: #forward but right is faster
                if last_state != state:
                    last_state = state
                    right_speed = mid_speed + diff_speed
                    left_speed = mid_speed
                    pub_L.publish(left_speed)
                    pub_R.publish(right_speed)
                else:
                    if sw_left and sw_right:
                        state = 1
                        back_situation = 3
                    elif sw_left:
                        back_situation = 2
                        state = 1
                    elif sw_right:
                        back_situation = 1
                        state = 1
                    
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
        except ValueError:
            pass

if __name__ == '__main__':
    main()



    