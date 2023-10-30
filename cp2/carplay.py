#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
def main():
    rospy.init_node('pwm_pub')
    pub_L = rospy.Publisher('motor_v_left', Int32, queue_size = 1)
    pub_R = rospy.Publisher('motor_v_right', Int32, queue_size = 1)
    # rospy.sleep(5)
    while not rospy.is_shutdown():
        try:
            left_speed = int(input("user's left is: "))
            right_speed = int(input("user's right is: "))
            pub_L.publish(left_speed)
            pub_R.publish(right_speed)
            rospy.sleep(0.5)
        except ValueError:
            pass
if __name__ == '__main__':
    main()