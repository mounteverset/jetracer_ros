#!/usr/bin/env python
import rospy
import math
import tf
import tf2_ros
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Pose
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class BallInsTor():
    def __init__(self):
        rospy.init_node("goal_scoring_node", anonymous=True)
        self.transform_listener = tf.TransformListener()
        self.ball_pose = TransformStamped()
        self.scoring_goal_pose = TransformStamped()
        # self.q1_inv = []
        # self.q2 = []
        self.r = 0.4
        self.fennec_goal_pose = Pose()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_goal = MoveBaseGoal()
        self.move_goal.target_pose.header.frame_id = "map"
        self.time_last_scoring_attempt = rospy.Time.now()
        self.throttle_pub = rospy.Publisher("throttle", Float32, queue_size=1)
        self.throttle_msg = Float32()
        self.full_throttle_time = 0.35

    def calculate_line(self):
        ball_pose_list = [self.ball_pose.transform.translation.x, self.ball_pose.transform.translation.y, self.ball_pose.transform.translation.z]
        ball_point = np.array(ball_pose_list, dtype=np.float)
        goal_pose_list =    [self.scoring_goal_pose.transform.translation.x, 
                             self.scoring_goal_pose.transform.translation.y, 
                             self.scoring_goal_pose.transform.translation.z]
        goal_point = np.array(goal_pose_list, dtype=np.float)
        goal_position = ball_point + self.r * (ball_point - goal_point)
        self.fennec_goal_pose.position.x = goal_position[0]
        self.fennec_goal_pose.position.y = goal_position[1]
        self.fennec_goal_pose.position.z = 0.0
    
    def calculate_orientation(self):

        fennec_goal_angle = math.atan(( self.ball_pose.transform.translation.y - self.scoring_goal_pose.transform.translation.y) / 
                                      ( self.ball_pose.transform.translation.x - self.scoring_goal_pose.transform.translation.y))
        fennec_goal_orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, fennec_goal_angle)
        self.fennec_goal_pose.orientation.x = fennec_goal_orientation[0]
        self.fennec_goal_pose.orientation.y = fennec_goal_orientation[1]
        self.fennec_goal_pose.orientation.z = fennec_goal_orientation[2]
        self.fennec_goal_pose.orientation.w = fennec_goal_orientation[3]

    def full_throttle(self):
        self.throttle_msg = -1.0
        self.throttle_pub.publish(self.throttle_msg)
        rospy.sleep(self.full_throttle_time)
        self.throttle_msg = 0.0
        self.throttle_pub.publish(self.throttle_msg)


    def goal_node(self):


        while not rospy.is_shutdown():

            # kurze Pause zwischen den einzelnen Schuessen
            #if (rospy.Time.now() - self.time_last_scoring_attempt) > 2:

            self.transform_listener.waitForTransform("/map", "/wilson_bag",  rospy.Time(0), rospy.Duration(15.0))
            self.transform_listener.waitForTransform("/map", "/ball", rospy.Time(0), rospy.Duration(15.0))
            # self.transform_listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(10.0))

            trans_goal, rot_goal = self.transform_listener.lookupTransform("map", "wilson_bag", rospy.Time(0))
            trans_ball, rot_ball = self.transform_listener.lookupTransform("map", "ball", rospy.Time(0))

            self.scoring_goal_pose.transform.translation.x = trans_goal[0]
            self.scoring_goal_pose.transform.translation.y = trans_goal[1]
            self.scoring_goal_pose.transform.translation.z = trans_goal[2]

            self.scoring_goal_pose.transform.rotation.x = rot_goal[0]
            self.scoring_goal_pose.transform.rotation.y = rot_goal[1]
            self.scoring_goal_pose.transform.rotation.z = rot_goal[2]
            self.scoring_goal_pose.transform.rotation.w = rot_goal[3]

            self.ball_pose.transform.translation.x = trans_ball[0]
            self.ball_pose.transform.translation.y = trans_ball[1]
            self.ball_pose.transform.translation.z = trans_ball[2]

            self.ball_pose.transform.rotation.x = rot_ball[0]
            self.ball_pose.transform.rotation.y = rot_ball[1]
            self.ball_pose.transform.rotation.z = rot_ball[2]
            self.ball_pose.transform.rotation.w = rot_ball[3]
            #trans_base, rot_base = self.transform_listener.lookupTransform("map", "base_link", rospy.Time(0))
            #trans_ball2goal, rot_ball2goal = self.transform_listener.lookupTransform("object_6", "ball", rospy.Time(0))

            self.calculate_line()
            self.calculate_orientation()

            #go_signal = rospy.wait_for_message("signal_sender", float)

            go_signal = 1

            if(go_signal == 1):
                
                self.client.wait_for_server()

                self.move_goal.target_pose.header.stamp = rospy.Time.now()

                self.move_goal.target_pose.pose = self.fennec_goal_pose

                # self.move_goal.target_pose.pose.orientation.x = 0
                # self.move_goal.target_pose.pose.orientation.y = 0
                # self.move_goal.target_pose.pose.orientation.z = 0
                # self.move_goal.target_pose.pose.orientation.w = 1
                # self.q1_inv[0] = rot_ball2goal.orientation.x
                # self.q1_inv[1] = rot_ball2goal.orientation.y
                # self.q1_inv[2] = rot_ball2goal.orientation.z
                # self.q1_inv[3] = -rot_ball2goal.orientation.w # Negate for inverse

                # qr = tf.transformations.quaternion_multiply(self.q2, self.q1_inv) 

                self.client.send_goal(self.move_goal)
                wait = self.client.wait_for_result()

                if not wait:
                    rospy.loginfo("Action server not available!")
                    rospy.signal_shutdown("Action server not available!")
                else:
                    self.full_throttle()
                    return self.client.get_result()


                   

if __name__ == '__main__':

    ballinstor = BallInsTor()
    ballinstor.goal_node()