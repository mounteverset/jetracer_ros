import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import pyrealsense2 as rs2

class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)

        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        self.sub_blobs = rospy.Subscriber("/blob/point_blob",Point, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # pick one pixel among all the pixels with the closest range:
            #indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
            #pix = (indices[1], indices[0])
            pix = (data.x,data.y)
            self.pix = pix
            line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])

            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            if (not self.pix_grade is None):
                line += ' Grade: %2d' % self.pix_grade
            line += '\r'
            sys.stdout.write(line)
            sys.stdout.flush()

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return



    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

def main():
    depth_image_topic = '/camera/depth/image_rect_raw'
    depth_info_topic = '/camera/depth/camera_info'

    print ('')
    print ('show_center_depth.py')
    print ('--------------------')
    print ('App to demontrate the usage of the /camera/depth topics.')
    print ('')
    print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    print ('Application then calculates and print the range to the closest object.')
    print ('If intrinsics data is available, it also prints the 3D location of the object')

    
    listener = ImageListener(depth_image_topic, depth_info_topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()
