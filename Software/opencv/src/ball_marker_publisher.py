#!/usr/bin/env python

import rospy
import cv2
import math
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker

class BallDepthGetter():

    def __init__(self):
        rospy.init_node("ball_marker_publisher", anonymous=True)
        self.cv_bridge = CvBridge()
        point = Point()
        point.x = 0
        point.y = 0
        point.z = 0
        self.blob_point = point
        self.camera_intrinsics = None
        self.depth = 0
        self.ball_pose = Pose()

        self.marker = Marker()
        self.marker.header.frame_id = "camera_link"
        self.marker.type = Marker.SPHERE
        self.marker.scale.x = 0.15
        self.marker.scale.y = 0.15
        self.marker.scale.z = 0.15       
        self.marker.color.b = 0.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.a = 1.0
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=5)

        # self.position_tolerance = 0.1
        
    def depth_callback(self, data):
        cv_image = self.cv_bridge.imgmsg_to_cv2(data, data.encoding)
        self.depth = cv_image[self.blob_point.y, self.blob_point.x] / 1000.0
        #rospy.loginfo("Got called in Depth Callback")
        rospy.loginfo("Depth: {0}".format(self.depth))


    def blob_point_callback(self, data):
        self.blob_point.x = int(data.x)
        self.blob_point.y = int(data.y)
        #rospy.loginfo("Got called in Blob Point Callback")
        rospy.loginfo("Blob Point x: {0} y:{1}".format(self.blob_point.x, self.blob_point.y))
    
    # def camera_info_callback(self, data):

    def calculate_angle(self):
        h_fov = 48.0
        v_fov = 40.0

        horizontal_angle = ((self.blob_point.x - 320) / 320.0) * (h_fov / 2.0)
        # dhraghae = self.blob_point.x - 320
        # rospy.loginfo(dhraghae)
        # drht = dhraghae/320.0
        # rospy.loginfo(drht)
        # horizontal_angle = drht * (h_fov * 0.5)
        rospy.loginfo("Winkel: {0}".format(horizontal_angle))
        return horizontal_angle

    def calculate_position(self, angle):
        x_coord = math.sin(math.radians(angle)) * self.depth
        y_coord = math.cos(math.radians(angle)) * self.depth
        rospy.loginfo("x: {0}, y: {1}".format(x_coord, y_coord))

        self.marker.pose.position.x = x_coord
        self.marker.pose.position.y = y_coord





    def ball_depth_node(self):
        depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        blob_pixel_sub = rospy.Subscriber("/blob/point_blob", Point, self.blob_point_callback)
        rate = rospy.Rate(2)
        #camera_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, camera_info_callback)
        while not rospy.is_shutdown():
            
            angle = self.calculate_angle()
            self.calculate_position(angle)
            self.marker_pub.publish(self.marker)
            rate.sleep()
        #rospy.spin()
            # rospy.loginfo("Depth: {0}".format(self.depth))
            # rospy.loginfo("Blob Point x: {0} y:{1}".format(self.blob_point.x, self.blob_point.y))


if __name__ == '__main__':
    print("Startin...")
    ball_depth_getter = BallDepthGetter()
    ball_depth_getter.ball_depth_node()