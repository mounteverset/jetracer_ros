#!/usr/bin/env python

import rospy
import tf
import tf2_ros

class BallBroadcaster():
    def __init__(self):
        rospy.init_node("ball_tf_broadcaster", anonymous=True)
        self.inital_detect = False
        self.broadcaster = tf.TransformBroadcaster()
        self.transform_listener = tf.TransformListener()
        self.ball_trans = []
        self.ball_rot = []


    def broadcaster_node(self):

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            try:
                self.transform_listener.waitForTransform("/map", "/blob", rospy.Time(), rospy.Duration(0.01))
                self.ball_trans, self.ball_rot = self.transform_listener.lookupTransform("/map", "/blob",rospy.Time())
                self.inital_detect = True
                print("X: {0}, Y: {1}".format(self.ball_trans[0], self.ball_trans[1]))
                if self.inital_detect == True:
                    self.broadcaster.sendTransform(self.ball_trans, self.ball_rot, rospy.Time.now(), "/ball", "/map")
                    
            except:
                print("Waiting for new ball transform timeout. Trying to publish old one.")
                if self.inital_detect == True:
                    self.broadcaster.sendTransform(self.ball_trans, self.ball_rot, rospy.Time.now(), "/ball", "/map")

            rate.sleep()



if __name__ == '__main__':
    broadcaster = BallBroadcaster()
    broadcaster.broadcaster_node()

