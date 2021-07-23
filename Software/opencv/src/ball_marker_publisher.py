#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo

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
        
    def depth_callback(self, data):
        cv_image = self.cv_bridge.imgmsg_to_cv2(data, data.encoding)
        self.depth = cv_image[self.blob_point.y, self.blob_point.x] / 10
        rospy.loginfo("Got called in Depth Callback")
        rospy.loginfo(self.depth)


    def blob_point_callback(self, data):
        self.blob_point.x = int(data.x)
        self.blob_point.y = int(data.y)
        rospy.loginfo("Got called in Blob Point Callback")
        rospy.loginfo("Blob Point x: {0} y:{1}".format(self.blob_point.x, self.blob_point.y))
    
    # def camera_info_callback(self, data):


    def ball_depth_node(self):
        depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        blob_pixel_sub = rospy.Subscriber("/blob/point_blob", Point, self.blob_point_callback)
        #camera_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, camera_info_callback)
        while not rospy.is_shutdown():
            rate = rospy.Rate(2)
            rate.sleep()
        #rospy.spin()
            # rospy.loginfo("Depth: {0}".format(self.depth))
            # rospy.loginfo("Blob Point x: {0} y:{1}".format(self.blob_point.x, self.blob_point.y))


if __name__ == '__main__':
    print("Startin...")
    ball_depth_getter = BallDepthGetter()
    ball_depth_getter.ball_depth_node()