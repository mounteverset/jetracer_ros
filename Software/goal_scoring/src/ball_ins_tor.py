#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class BallInsTor():
    def __init__(self):
        rospy.init_node("BallInsTor", anonymous=True)
        self.transform_listener = tf.TransformListener()
        self.q1_inv = []
        self.q2 = []

    def berechneGerade(self, trans_ball, trans_goal, r):
        ball_point = np.array(trans_ball, dtype=np.float)
        goal_point = np.array(trans_goal, dtype=np.float)
        gl = ball_point + r * (ball_point - goal_point)

        return gl

    def goal_node(self):

        while not rospy.is_shutdown():
            self.transform_listener.waitForTransform("/map", "/object_6",  rospy.Time(0), rospy.Duration(10.0))
            self.transform_listener.waitForTransform("/map", "/ball", rospy.Time(0), rospy.Duration(10.0))
            self.transform_listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(10.0))

            trans_goal, rot_goal = self.transform_listener.lookupTransform("map", "object_6", rospy.Time(0))
            trans_ball, rot_ball = self.transform_listener.lookupTransform("map", "ball", rospy.Time(0))
            #trans_base, rot_base = self.transform_listener.lookupTransform("map", "base_link", rospy.Time(0))
            #trans_ball2goal, rot_ball2goal = self.transform_listener.lookupTransform("object_6", "ball", rospy.Time(0))

            result = self.berechneGerade(trans_ball, trans_goal, 1.2)

            #go_signal = rospy.wait_for_message("signal_sender", float)
            go_signal = 1
            if(go_signal == 1):
                client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                client.wait_for_server()

                move_goal = MoveBaseGoal()
                move_goal.target_pose.header.frame_id = "map"
                move_goal.target_pose.header.stamp = rospy.Time.now()

                move_goal.target_pose.pose.position.x = result[0]
                move_goal.target_pose.pose.position.y = result[1]
                move_goal.target_pose.pose.position.z = 0

                move_goal.target_pose.pose.orientation.x = 0
                move_goal.target_pose.pose.orientation.y = 0
                move_goal.target_pose.pose.orientation.z = 0
                move_goal.target_pose.pose.orientation.w = 1
                # self.q1_inv[0] = rot_ball2goal.orientation.x
                # self.q1_inv[1] = rot_ball2goal.orientation.y
                # self.q1_inv[2] = rot_ball2goal.orientation.z
                # self.q1_inv[3] = -rot_ball2goal.orientation.w # Negate for inverse

                # qr = tf.transformations.quaternion_multiply(self.q2, self.q1_inv) 

                client.send_goal(move_goal)
                wait = client.wait_for_result()

                if not wait:
                    rospy.loginfo("Action server not available!")
                    rospy.signal_shutdown("Action server not available!")
                else:
                    return client.get_result()


if __name__ == '__main__':

    ballinstor = BallInsTor()
    ballinstor.goal_node()