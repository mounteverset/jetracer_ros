#!/usr/bin/env python3

import rospy, math
from jetracer.nvidia_racecar import NvidiaRacecar
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
#from ackerman_msgs.msg import AckermanDrive, AckermanDriveStamped

class Jetracer():
    def __init__(self):
        self.car = NvidiaRacecar()
        self.car.steering_gain = 0.8
        self.car.steering_offset = 0.16 #-0.16
        self.car.throttle_gain = 1
        self.car.steering = 0.0
        self.car.throttle = 0.0
        self.wheelbase = 0.2

    def convert_rotational_vel_to_steering_angle(self, v, omega):
        if v == 0 or omega == 0:
            return 0
        
        radius = v / omega
        return math.atan(self.wheelbase/radius)

    def callback_cmd_vel(self, cmd_vel):
        speed = cmd_vel.linear.x
        steering = self.convert_rotational_vel_to_steering_angle(speed, cmd_vel.angular.z)

        if speed > 1:
            self.car.throttle = 1.0
        elif speed < -1:
            self.car.throttle = -1.0
        else:
            self.car.throttle = speed

        if steering > 1:
            self.car.steering = 1.0
        elif steering < -1:
            self.car.steering = -1.0
        else:
            self.car.steering = steering

    def throttle_callback(self, data):
        if data.data > 1:
            self.car.throttle = 1.0
        elif data.data < -1:
            self.car.throttle = -1.0
        else:
            self.car.throttle = data.data

    def steering_callback(self, data):
        if data.data > 1:
            self.car.steering = 1.0
        elif data.data < -1:
            self.car.steering = -1.0
        else:
            self.car.steering = data.data  

    #Setup node and topics subscription
    def racecar_node(self):
        
        rospy.init_node('racecar', anonymous=True)
        #rospy.Subscriber("throttle", Float32, callback_throttle)
        #rospy.Subscriber("steering", Float32, callback_steering)
        rospy.Subscriber("cmd_vel", Twist, self.callback_cmd_vel)
        rospy.Subscriber("throttle", Float32, self.throttle_callback, queue_size=1)
        rospy.Subscriber("steering", Float32, self.steering_callback, queue_size=1)
        
        #rospy.Subscriber("ackerman_control/odom", Odometry, callback_odom)

        rospy.spin()

if __name__ == '__main__':
    print("Running racecar.py")
    jetracer = Jetracer()
    jetracer.racecar_node()