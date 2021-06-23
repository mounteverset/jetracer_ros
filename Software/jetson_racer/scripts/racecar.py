#!/usr/bin/env python3

import rospy, math
from jetracer.nvidia_racecar import NvidiaRacecar
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
#from ackerman_msgs.msg import AckermanDrive, AckermanDriveStamped

car = NvidiaRacecar()
car.steering_gain = 0.65
car.steering_offset = 0.12
car.throttle_gain = 1
car.steering = 0.0
car.throttle = 0.0

def convert_rotational_vel_to_steering_angle(v, omega, wheelbase):
    if v == 0 or omega == 0:
        return 0
    
    radius = v / omega
    return math.atan(wheelbase/radius)

def callback_cmd_vel(cmd_vel):
    speed = cmd_vel.linear.x
    steering = convert_rotational_vel_to_steering_angle(speed, cmd_vel.angular.z, 30)

    if speed > 1:
        car.throttle = 1.0
    elif speed < -1:
        car.throttle = -1.0
    else:
        car.throttle = speed

    if steering > 1:
        car.steering = 1.0
    elif steering < -1:
        car.steering = -1.0
    else:
        car.steering = steering

def throttle_callback(data):
    if data.data > 1:
        car.throttle = 1.0
    elif data.data < -1:
        car.throttle = -1.0
    else:
        car.throttle = data.data

def steering_callback(data):
    if data.data > 1:
        car.steering = 1.0
    elif data.data < -1:
        car.steering = -1.0
    else:
        car.steering = data.data  

#Setup node and topics subscription
def racecar():
    rospy.init_node('racecar', anonymous=True)
    #rospy.Subscriber("throttle", Float32, callback_throttle)
    #rospy.Subscriber("steering", Float32, callback_steering)
    rospy.Subscriber("cmd_vel", Twist, callback_cmd_vel)
    rospy.Subscriber("throttle", Float32, throttle_callback, queue_size=1)
    rospy.Subscriber("steering", Float32, steering_callback, queue_size=1)
    
    #rospy.Subscriber("ackerman_control/odom", Odometry, callback_odom)

    rospy.spin()

if __name__ == '__main__':
    print("Running racecar.py")
    racecar()
