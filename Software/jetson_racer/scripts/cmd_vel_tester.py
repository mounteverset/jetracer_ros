#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rospy

def cmd_vel_test(vel):
    rospy.init_node('tester', anonymous=True)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    twist_msg = Twist()
    rospy.sleep(0.1)
    twist_msg.linear.x = vel
    print("test")
    cmd_pub.publish(twist_msg)
    

if __name__ == '__main__':
    cmd_vel_test(0.20)
    rospy.sleep(0.9)
    cmd_vel_test(0)
    
# 0.5 @ 1s = 100cm
# 1   @ 1s = 210cm
