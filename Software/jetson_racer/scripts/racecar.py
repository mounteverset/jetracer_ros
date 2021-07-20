#!/usr/bin/env python3

import rospy, math
from numpy import interp
from jetracer.nvidia_racecar import NvidiaRacecar
from std_msgs.msg import Float32, Float64MultiArray
from geometry_msgs.msg import Twist, TwistStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
#from tf2_ros import TransformBroadcaster


from geometry_msgs.msg import TransformStamped
#from ackerman_msgs.msg import AckermanDrive, AckermanDriveStamped

class Jetracer():
    def __init__(self):
        self.car = NvidiaRacecar()
        self.car.steering_gain = 0.8
        self.car.steering_offset = 0.16 #-0.16
        self.car.throttle_gain = 1
        self.car.steering = 0.0
        self.car.throttle = 0.0

        rospy.init_node('racecar', anonymous=True)
        self.angular_vel = 0.0
        self.wheelbase = 0.16
        self.pose = Pose()
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.throttle_steer_pub = rospy.Publisher("/racecar_data", Float64MultiArray, queue_size=2)
        #self.steering_pub = rospy.Publisher("/steering", Float32, queue_size=2)
        #self.odom_broadcaster = tf.broadcaster.TransformBroadcaster()
        #self.last_timestamp = rospy.Time.now()
        self.max_steering_angle = 0.366
        self.min_speed = 0.29


    # def calculate_odometry(self, cmd_vel):
    #     # distance traveled in x per second = 208 * cmd_vel.linear.x - 43.64
    #     # Auto bewegt sich nicht bei cmd_vel.linear.x Werten unter 0.2 -> zurückgelegte Strecke 0
    #     duration = (rospy.Time.now() - self.last_timestamp).to_sec()
    #     self.last_timestamp = rospy.Time.now()
    #     euler = tf.transformations.euler_from_quaternion(self.pose.orientation)
    #     x_dot = 0
    #     y_dot = 0

    #     if self.car.throttle == 0:
    #         x_dot = 0
    #         y_dot = 0
    #     else: 
    #         x_dot = (208 * self.car.throttle - 43) * math.cos(euler[2]) # euler[2] == yaw
    #         y_dot = (208 * self.car.throttle -43) * math.sin(euler[2])

    #     self.pose.position.x += x_dot * duration
    #     self.pose.position.y += y_dot * duration

    #     euler[2] += self.angular_vel * duration # update yaw 
    #     self.pose.orientation = tf.transformations.quaternion_from_euler(euler)

    #     odom = Odometry()
    #     odom.header.frame_id = "/odom"
    #     odom.header.stamp = rospy.Time.now()
    #     odom.pose = self.pose
    #     odom.twist.covariance[0] = 0.2 #x
    #     odom.twist.covariance[7] = 0.2 #y 
    #     odom.twist.covariance[35] = 0.4 #yaw
    #     odom.twist.twist.linear.x = self.car.throttle
    #     odom.twist.twist.linear.y = 0
    #     odom.twist.twist.angular.z = self.angular_vel


    #     # Odom Transform publishen

    #     odom_stamped = TransformStamped()
    #     odom_stamped.header.stamp = self.last_timestamp
    #     odom_stamped.header.frame_id = "odom"
    #     odom_stamped.child_frame_id = "base_link"
    #     odom_stamped.transform.translation.x = x_dot * duration
    #     odom_stamped.transform.translation.y = y_dot * duration


    #     return odom

    #def broadcast_odom_tf(self, odom):

    def convert_rotational_vel_to_steering_angle(self, v, omega):
        if omega == 0 or v == 0:
            return 0
        
        radius = v / omega
        print("Radius : {0}".format(radius))
        print("Steering Angle in Radian: {0}".format(math.atan(self.wheelbase/radius)))
        print("Steering Angle in Degree: {0}".format(math.degrees(math.atan(self.wheelbase/radius))))
        return math.atan(self.wheelbase/radius)

    def callback_cmd_vel(self, cmd_vel):

        # odom = self.calculate_odometry(cmd_vel)
        # self.odom_pub.publish(odom)
            
        # speed = -cmd_vel.linear.x

        # # cmd_vel.lin.x Werte zwischen -0.2 und 0.2 bewegen das Auto nicht.
        # if speed > -self.min_speed:
        #     if speed < 0:
        #         speed = -self.min_speed
        #     elif self.min_speed > speed > 0:
        #         speed = self.min_speed
        #self.angular_vel = cmd_vel.angular.z

        speed = cmd_vel.linear.x

        if abs(speed) < self.min_speed:
            if speed < 0:
                speed = -self.min_speed
            elif speed > 0:
                speed = self.min_speed
        
        speed = -speed

        steering_angle = self.convert_rotational_vel_to_steering_angle(speed, cmd_vel.angular.z)
        print("Steering Angle pre cut: {0}".format(steering_angle))
        if steering_angle < -self.max_steering_angle:
            steering_angle = -self.max_steering_angle
        elif steering_angle > self.max_steering_angle:
            steering_angle = self.max_steering_angle

        print("Steering Angle after cut: {0}".format(steering_angle))
        steering_command = interp(steering_angle, [-self.max_steering_angle, self.max_steering_angle], [-1,1])

        print("Steering command send to Jetracer: {0}".format(steering_command))

    if speed > 1:
        car.throttle = 1.0
    elif speed < -1:
        car.throttle = -1.0
    else:
        car.throttle = speed

        if steering_command > 1:
            self.car.steering = 1.0
        elif steering_command < -1:
            self.car.steering = -1.0
        else:
            self.car.steering = steering_command  

        self.angular_vel = (self.car.throttle * math.tan(steering_angle)) / self.wheelbase  

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
    def racecar_node(self):
        
        # throttle = Float32()
        # steering = Float32()
        racecar_data = []
        racecar_data.append(self.car.throttle)
        racecar_data.append(self.car.steering)
        racecar_data.append(self.angular_vel)
        racecar_data_msg = Float64MultiArray()

        #rospy.Subscriber("throttle", Float32, callback_throttle)
        #rospy.Subscriber("steering", Float32, callback_steering)
        rospy.Subscriber("cmd_vel", Twist, self.callback_cmd_vel)
        rospy.Subscriber("throttle", Float32, self.throttle_callback, queue_size=1)
        rospy.Subscriber("steering", Float32, self.steering_callback, queue_size=1)
        print("Startup complete.")
        #rospy.Subscriber("ackerman_control/odom", Odometry, callback_odom)
        while not rospy.is_shutdown():
        #    odom = self.calculate_odometry(self)
            
            # throttle.data = self.car.throttle
            # steering.data = self.car.steering
            racecar_data[0] = self.car.throttle
            racecar_data[1] = self.car.steering
            racecar_data[2] = self.angular_vel
            racecar_data_msg.data = racecar_data
            self.throttle_steer_pub.publish(racecar_data_msg)
            # self.throttle_steer_pub.publish(throttle)
            # self.steering_pub.publish(steering)
            # rospy.sleep(0.05)
            # rate.sleep()

        
        #rospy.spin()

if __name__ == '__main__':
    print("Running racecar.py")
    jetracer = Jetracer()
    jetracer.racecar_node()
