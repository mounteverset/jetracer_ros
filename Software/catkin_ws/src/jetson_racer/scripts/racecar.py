#!/usr/bin/env python3

from joint_state import callback_cmd
import rospy
from jetracer.nvidia_racecar import NvidiaRacecar
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

#Initialize car variable and tune settings
car = NvidiaRacecar()
car.steering_gain = 0.65
car.steering_offset = -0.16
car.throttle_gain = 1
car.steering = 0.0
car.throttle = 0.0

# #Throttle
# def callback_throttle(throt):
#     car.throttle = throt.data
#     rospy.loginfo("Throttle: %s", str(throt.data))

# #Steering
# def callback_steering(steer):
#     car.steering = steer.data
#     rospy.loginfo("Steering: %s", str(steer.data))

#Throttle and Steering read from Twist Msg
def callback_cmd(cmd):
    car.throttle = cmd.linear.x
    car.steering = cmd.angular.z

#Odometry readings added on top of Throttle to keep the wheels moving while having drastic velocity changes (otherwise instant halt)
def callback_odom(odom):
    car.throttle = car.throttle + odom.twist.twist.linear.x #vielleicht hier ein fehler?
    car.steering = odom.twist.twist.angular.z

#Setup node and topics subscription
def racecar():
    rospy.init_node('racecar', anonymous=True)
    #rospy.Subscriber("throttle", Float32, callback_throttle)
    #rospy.Subscriber("steering", Float32, callback_steering)
    rospy.Subscriber("ackerman_control/cmd_vel", Twist, callback_cmd)
    rospy.Subscriber("ackerman_control/odom", Odometry, callback_odom)

    rospy.spin()

if __name__ == '__main__':
    print("Running racecar.py")
    racecar()
