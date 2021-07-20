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
        self.marker.header.frame_id = "map"
        self.marker.type = marker.CUBE
        self.marker.scale.x = 0.8
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.5
        self.position_tolerance = 0.1

    def goal_node(self):

        while not rospy.is_shutdown():
            self.transform_listener.waitForTransform("/object_goal", "/map", rospy.Time(), rospy.Duration(4.0))
            trans, rot = tf.lookupTransform("object_goal", "map", rospy.Time.now())

            if abs(self.Pose.position.x - trans[0]) > self.position_tolerance or abs(self.Pose.position.y - trans[1]) > self.position_tolerance:
                self.Pose.position.x = trans[0]
                self.Pose.position.y = trans[1]
                self.Pose.position.z = trans[2]
                self.Pose.orientation.x = rot[0]
                self.Pose.orientation.y = rot[1]
                self.Pose.orientation.z = rot[2]
                self.Pose.orientation.w = rot[3]

                self.marker.pose = self.Pose
                rospy.sleep(0.5)
                self.marker_pub.publish(self.marker)
            


            
    

if __name__ == '__main__':
    print("Running Goal Marker Publisher.")
    
    pose_publisher = Goal_Publisher()
    Goal_Publisher.goal_node()


 pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 100)
        markerArray = MarkerArray()

        marker = Marker()
        marker.id = markerid
        marker.header.frame_id = "world"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.pose.position.x = pose_goal.pose.position.x
        marker.pose.position.y = pose_goal.pose.position.y
        marker.pose.position.z = pose_goal.pose.position.z
        marker.pose.orientation.x = pose_goal.pose.orientation.x
        marker.pose.orientation.y = pose_goal.pose.orientation.y
        marker.pose.orientation.z = pose_goal.pose.orientation.z
        marker.pose.orientation.w = pose_goal.pose.orientation.w
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        markerArray.markers.append(marker)
        rospy.sleep(3)
        pub.publish(markerArray)