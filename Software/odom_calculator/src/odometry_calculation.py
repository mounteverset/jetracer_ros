#!/usr/bin/env python

import rospy, math
from numpy import interp
from std_msgs.msg import Float32, Float64MultiArray
from geometry_msgs.msg import Twist, TwistStamped, Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf



class Odometry_Calculator:
    def __init__(self):
        ## tbd 
        rospy.init_node("odometry_calculator", anonymous=True)
        self.throttle = 0.0
        self.steering = 0.0
        self.pose = Pose()
        self.angular_vel = 0.0
        self.last_timestamp = rospy.Time.now()
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_trans_broadcaster = TransformBroadcaster()

    def calculate_odometry(self):
        # distance traveled in x per second = 208 * cmd_vel.linear.x - 43.64
        # Auto bewegt sich nicht bei cmd_vel.linear.x Werten unter 0.2 -> zurueckgelegte Strecke 0
        duration = (rospy.Time.now() - self.last_timestamp).to_sec()
        print("Duration between calculation: {0}".format(duration))
        self.last_timestamp = rospy.Time.now()

        explicit_quat = [
                        self.pose.orientation.x,
                        self.pose.orientation.y,
                        self.pose.orientation.z,
                        self.pose.orientation.w
                        ]

        print("Position: ( {0} | {1} | {2} ) \n Orientation: ( {3} | {4} | {5} | {6}".format(
            self.pose.position.x, 
            self.pose.position.y,
            self.pose.position.z,
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ))

        #explicit_quat = [0.0, 0.0, 0.0, 0.0]

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
        print("Yaw: {0}".format(yaw))
        x_dot = 0
        y_dot = 0

        if self.throttle == 0:
            x_dot = 0
            y_dot = 0
        else: 
            x_dot = (208 * self.throttle - 43) * math.cos(yaw) # euler[2] == yaw
            y_dot = (208 * self.throttle -43) * math.sin(yaw)

        self.pose.position.x += x_dot * duration
        self.pose.position.y += y_dot * duration

        yaw += self.angular_vel * duration # update yaw 

        
        explicit_quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        #Reihenfolge von x y z w unklar im nparray was von quaternion_from_euler zurueckkomt -> pruefen
        self.pose.orientation.x = explicit_quat[0]
        self.pose.orientation.y = explicit_quat[1]
        self.pose.orientation.z = explicit_quat[2]
        self.pose.orientation.w = explicit_quat[3]

        odom = Odometry()
        odom.header.frame_id = "/odom"
        odom.header.stamp = rospy.Time.now()
        odom.pose = self.pose
        odom.twist.covariance[0] = 0.2 #x
        odom.twist.covariance[7] = 0.2 #y 
        odom.twist.covariance[35] = 0.4 #yaw
        odom.twist.twist.linear.x = self.throttle
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = self.angular_vel


        # Odom Transform publishen

        odom_stamped = TransformStamped()
        odom_stamped.header.stamp = self.last_timestamp
        odom_stamped.header.frame_id = "odom"
        odom_stamped.child_frame_id = "base_link"
        odom_stamped.transform.translation.x = x_dot * duration
        odom_stamped.transform.translation.y = y_dot * duration
        self.odom_trans_broadcaster.sendTransform(odom_stamped)

        return odom

    def throttle_callback(self, throttle):
        self.throttle = throttle.data

    def steering_callback(self, steering):
        self.steering = steering.data

    def racecar_data_callback(self, racecar_data):
        self.throttle = racecar_data.data[0]
        self.steering = racecar_data.data[1]
        self.angular_vel = racecar_data.data[2]
    


    def odometry_node(self):
        # rospy.Subscriber("throttle", Float32, self.throttle_callback, queue_size=1)
        # rospy.Subscriber("steering", Float32, self.steering_callback, queue_size=1)
        rospy.Subscriber("racecar_data", Float64MultiArray, self.racecar_data_callback, queue_size=1)
        print("Odometry started.")
        while not rospy.is_shutdown():
            odom = self.calculate_odometry()
            self.odom_pub.publish(odom)

            rospy.sleep(0.1)

        #rospy.spin()
    
    # def convert_rotational_vel_to_steering_angle(self, v, omega):
    #     if omega == 0:
    #         return 0
        
    #     radius = v / omega
    #     print("Radius : {0}".format(radius))
    #     print("Steering Angle in Radian: {0}".format(math.atan(self.wheelbase/radius)))
    #     print("Steering Angle in Degree: {0}".format(math.degrees(math.atan(self.wheelbase/radius))))
    #     return math.atan(self.wheelbase/radius)


if __name__ == '__main__':
    
    odom_calc = Odometry_Calculator()
    odom_calc.odometry_node()
    print("Shutting down")

