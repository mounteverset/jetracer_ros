#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels
Subscribes to 
    /blob/point_blob
    
Publishes commands to 
    /dkcar/control/cmd_vel    
"""
import math, time
import rospy
from std_msgs import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

K_LAT_DIST_TO_STEER     = 2.0

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)

class ChaseBall():
    def __init__(self):
        
        self.blob_x         = 0.0
        self.blob_y         = 0.0
        self._time_detected = 0.0
        
        self.sub_center = rospy.Subscriber("/blob/point_blob", Point, self.update_ball)
        rospy.loginfo("Subscribers set")
        
        self.throttle_pub = rospy.Publisher("throttle", Float32, queue_size=1)
        self.steering_pub = rospy.Publisher("steering", Float32, queue_size=1)

        # Temporarily not active to try to direclty publish motor commands
        # self.pub_twist = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        
        # self._message = Twist()
        self._steering_msg = Float32()
        self._throttle_msg = Float32()
        
        self._time_steer        = 0
        self._steer_sign_prev   = 0
        
    @property
    def is_detected(self): return(time.time() - self._time_detected < 1.0)
        
    def update_ball(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        self._time_detected = time.time()
        rospy.loginfo("Ball detected: %.1f  %.1f "%(self.blob_x, self.blob_y))

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        
        Steer will be added to the commanded throttle
        throttle will be multiplied by the commanded throttle
        """
        steer_action    = 0.0
        throttle_action = 0.0
        
        if self.is_detected:
            #--- Apply steering, proportional to how close is the object
            steer_action   = -K_LAT_DIST_TO_STEER*self.blob_x
            steer_action   = saturate(steer_action, 1.0, -1.0)
            rospy.loginfo("Steering command %.2f"%steer_action) 
            throttle_action = -0.27
            
        return (steer_action, throttle_action)
        
    def run(self):
        
        #--- Set the control rate
        # default: rospy.Rate(5)
        rate = rospy.Rate(40)

        while not rospy.is_shutdown():
            #-- Get the control action
            steer_action, throttle_action    = self.get_control_action() 
            
            rospy.loginfo("Steering = %3.1f"%(steer_action))
            
            #-- update the message
            self._throttle_msg = throttle_action
            self._steering_msg = steer_action
            # self._message.linear.x  = throttle_action
            # self._message.angular.z = -steer_action
            
            #-- publish it
            # Twist pub temp deactivated
            # self.pub_twist.publish(self._message)
            self.throttle_pub.publish(self._throttle_msg)
            self.steering_pub.publish(self._steering_msg)


            rate.sleep()        
            
if __name__ == "__main__":

    rospy.init_node('chase_ball')
    
    chase_ball = ChaseBall()
    chase_ball.run()            


# if self.Umut == "toxic":
#     instant_forfeit()
    
#     for i in range(3):
#         print("What a save!")

#     print("Chat has been diasbled for 3 seconds.")

