#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

import math


rear_vel = 0
front_rot = 0

#Throttle
def callback_throttle(throt):
    global rear_vel
    rear_vel = rear_vel + throt.data*0.01

#Steering
def callback_steering(steer):
    global front_rot
    front_rot = steer.data*-0.4

def callback_cmd(cmd):
    global front_rot
    global rear_vel
    rear_vel = rear_vel + cmd.linear.x
    front_rot = cmd.angular.z



def talker():
    global rear_vel
    global front_rot

    #Initialize node and publisher
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    rospy.init_node('racecar_joint_state')
    rate = rospy.Rate(10) # 10hz



    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.1
    vy = -0.1
    vth = 0.1


    
    current_time = rospy.Time()
    last_time = rospy.Time()


    #Create joint state message
    joint = JointState()
    joint.header = Header()
    joint.name = ['front_left_steer_joint', 'front_left_wheel_joint','front_right_steer_joint', 'front_right_wheel_joint'\
	, 'rear_left_wheel_joint', 'rear_right_wheel_joint']
    joint.velocity = [0, 0, 0, 0, 0, 0]
    joint.effort = []

    while not rospy.is_shutdown():


        rospy.Subscriber("ackerman_control/cmd_vel", Twist, callback_cmd)

        #Pubblish joint state for Rviz
        joint.header.stamp = rospy.Time.now()
        joint.position = [front_rot, rear_vel, front_rot, rear_vel,  rear_vel, rear_vel]
        pub.publish(joint)

        rate.sleep()

if __name__ == '__main__':
    try:
        print("Running joint_state.py...")
        talker()
    except rospy.ROSInterruptException:
        pass
