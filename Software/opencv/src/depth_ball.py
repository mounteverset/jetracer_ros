#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import pyrealsense2
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image




class ball_depth:

    def __init__(self):
        self.sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.imageDepth_Callback)
        #self.sub_info = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.cameraInfo_Callback)
        self.sub_pixel = rospy.Subscriber("/blob/point_blob", Point, self.catchPixel_Callback)
        self.cameraInfos = rospy.wait_for_message("/camera/aligned_depth_to_color/camera_info", CameraInfo)
        self.x = None
        self.y = None
        #self.depth = rospy.wait_for_message("/camera/depth/image_rect_raw", Image)
        self.depth = None
        self.result = None

    def cameraInfo_Callback(self, data):
        self.cameraInfos = data

    def catchPixel_Callback(self, coordinate):
        x_0 = coordinate.x
        y_0 = coordinate.y

        #rospy.loginfo(y_0)
        #rospy.loginfo(x_0)
        self.x = int(x_0)
        self.y = int(y_0)

    def imageDepth_Callback(self, data):
        self.bridge = CvBridge()
        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        pix = (self.y, self.x)
        
        #rospy.loginfo(self.x)
        self.depth = cv_image[pix[0], pix[1]]
        if(self.depth != 0):
            rospy.loginfo (self.depth)
        #rospy.loginfo (type(self.depth))

        
        
    def depth_to_xy(self, x, y, depth, cameraInfo):
        
        self.intrinsics = pyrealsense2.intrinsics()
        self.intrinsics.width = cameraInfo.width
        self.intrinsics.height = cameraInfo.height
        self.intrinsics.ppx = cameraInfo.K[2]
        self.intrinsics.ppy = cameraInfo.K[5]
        self.intrinsics.fx = cameraInfo.K[0]
        self.intrinsics.fy = cameraInfo.K[4]
        
        #self.intrinsics.model = None
        self.intrinsics.coeffs = [i for i in cameraInfo.D]
        if(depth != 0):
            self.result = pyrealsense2.rs2_deproject_pixel_to_point(self.intrinsics, [x,y], depth)
            rospy.loginfo(self.result)




if __name__ == '__main__':
    print("Starting main")
    rospy.init_node("depth_to_object", anonymous=True)
    objekt = ball_depth()
    #objekt.depth_to_xy(objekt.x, objekt.y, objekt.depth, objekt.cameraInfos)
    rospy.spin()
