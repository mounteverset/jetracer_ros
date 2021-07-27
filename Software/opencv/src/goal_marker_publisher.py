#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker

class Goal_Publisher():
    def __init__(self):
        rospy.init_node("goal_pose_publisher", anonymous=True)
        self.last_timestamp = rospy.Time.now()
        self.goal_detected = False
        self.Pose = Pose()        
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size= 5)
        self.transform_listener = tf.TransformListener()
        self.marker = Marker()
        self.marker.id = 0
        self.marker.header.frame_id = "map"
        self.marker.type = Marker.CUBE
        self.marker.scale.x = 0.15
        self.marker.scale.y = 0.75
        self.marker.scale.z = 0.35
        self.position_tolerance = 0.1
        self.marker.color.b = 0.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.a = 1.0
        self.broadcaster = tf.TransformBroadcaster()

    def goal_node(self):
        

        while not rospy.is_shutdown():
            trans = []
            rot = []
            try:
                self.transform_listener.waitForTransform("/map", "/object_6", rospy.Time(), rospy.Duration(2.0))
                trans, rot = self.transform_listener.lookupTransform("/map", "/object_6",rospy.Time())
                self.goal_detected = True
                
                if len(trans) > 0:   
                    if abs(self.Pose.position.x - trans[0]) > self.position_tolerance or abs(self.Pose.position.y - trans[1]) > self.position_tolerance:
                        self.Pose.position.x = trans[0]
                        self.Pose.position.y = trans[1]
                        self.Pose.position.z = 0.35 / 2
                        self.Pose.orientation.x = 0.0 # rot[0]
                        self.Pose.orientation.y = 0.0 # rot[1]
                        self.Pose.orientation.z = rot[2] 
                        self.Pose.orientation.w = rot[3]

                        self.marker.pose = self.Pose
                        rospy.sleep(1.0)
                        self.marker_pub.publish(self.marker)

                if self.goal_detected == True:
                    # roll, pitch, yaw = tf.transformations.euler_from_quaternion([self.Pose.orientation.x, self.Pose.orientation.y, self.Pose.orientation.z, self.Pose.orientation.w])
                    # roll = 0
                    # pitch = 0
                    # orientation
                    self.broadcaster.sendTransform([self.Pose.position.x, self.Pose.position.y, 0.0], 
                                                    [0.0, 0.0, self.Pose.orientation.z, self.Pose.orientation.w], 
                                                    rospy.Time.now(), 
                                                    "wilson_bag",
                                                    "map")

            except:
                print("Waiting for transform timeout.")
                if self.goal_detected == True:
                    # roll, pitch, yaw = tf.transformations.euler_from_quaternion([self.Pose.orientation.x, self.Pose.orientation.y, self.Pose.orientation.z, self.Pose.orientation.w])
                    # roll = 0
                    # pitch = 0
                    # orientation
                    self.broadcaster.sendTransform([self.Pose.position.x, self.Pose.position.y, 0.0], 
                                                    [0.0, 0.0, self.Pose.orientation.z, self.Pose.orientation.w], 
                                                    rospy.Time.now(), 
                                                    "wilson_bag",
                                                    "map")

            # try: 
                
            # except:
            #     print("No transform.")
            #     continue

            if len(trans) > 0:   
                if abs(self.Pose.position.x - trans[0]) > self.position_tolerance or abs(self.Pose.position.y - trans[1]) > self.position_tolerance:
                    self.Pose.position.x = trans[0]
                    self.Pose.position.y = trans[1]
                    self.Pose.position.z = 0.35 / 2
                    self.Pose.orientation.x = 0.0 # rot[0]
                    self.Pose.orientation.y = 0.0 # rot[1]
                    self.Pose.orientation.z = rot[2] 
                    self.Pose.orientation.w = rot[3]

                    self.marker.pose = self.Pose
                    rospy.sleep(1.0)
                    self.marker_pub.publish(self.marker)
            
            if self.goal_detected == True:
                self.marker_pub.publish(self.marker)

            rospy.sleep(1.0)

            

if __name__ == '__main__':
    print("Running Goal Marker Publisher.")
    
    pose_publisher = Goal_Publisher()
    pose_publisher.goal_node()