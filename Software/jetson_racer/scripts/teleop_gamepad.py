#!/usr/bin/env python3

# import rospy
# import pygame
# import time
# from std_msgs.msg import Float32
# from geometry_msgs.msg import Twist, TwistStamped

# #Initialize pygame and gamepad
# pygame.init()
# j = pygame.joystick.Joystick(0)
# j.init()
# print ('Initialized Joystick : %s' % j.get_name())



# def teleop_gamepad():

#     #Setup topics publishing and nodes
#     #pub_throttle = rospy.Publisher('throttle', Float32, queue_size=8)
#     #pub_steering = rospy.Publisher('steering', Float32, queue_size=8)
#     pub_ackerman = rospy.Publisher('ackerman_control/cmd_vel', Twist, queue_size=10)
#     rospy.init_node('teleop_gamepad', anonymous=True)
#     rate = rospy.Rate(10) # 10hz

#     twist_msg = Twist()
#     while not rospy.is_shutdown():
#         pygame.event.pump()
        
#         #Obtain gamepad values
#         twist_msg.linear.x = j.get_axis(1) #Left thumbstick Y
#         twist_msg.angular.z = j.get_axis(2) #Right thumbstick X
#         print("Throttle:", twist_msg.linear.x)
#         print("Steering:", twist_msg.angular.z)

#         #Pubblish gamepad values
#         #pub_throttle.publish(rear_vel)
#         #pub_steering.publish(front_rot)
#         pub_ackerman.publish(twist_msg)
        
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         teleop_gamepad()
#     except rospy.ROSInterruptException:
#         pass



# import rospy
# #import pygame
# import time
# from inputs import devices, get_gamepad, GamePad
# from std_msgs.msg import Float32
# from geometry_msgs.msg import Twist, TwistStamped


# gamepadd = devices.gamepads[0]


# def teleop_gamepad():

#     #Setup topics publishing and nodes
#     rospy.init_node('teleop_gamepad', anonymous=True)

#     pub_ackerman = rospy.Publisher('ackerman_control/cmd_vel', Twist, queue_size=10)

#     rate = rospy.Rate(1000) # 10hz


#     twist_msg = Twist()
#     save_event = 0

#     while not rospy.is_shutdown():


#         gamepad = gamepadd.read()
    
#         for event in gamepad:
#             print(event.ev_type)
#             print("!!!!!!!!!!!!!!!!!")
#             print(event.code)
#             print("!!!!!!!!!!!!!!!!!")
#             print(event.state)
            
#             if (event.code == "ABS_Y"):
#                 #throttle = (event.state - 128) / 128
#                 save_event = -10*((event.state - 128) / 128)
#                 twist_msg.linear.x = save_event
#                 #print ("Gas: {0}".format(twist_msg.linear.x))
                
#             elif (event.code == "ABS_Z"):
#                 #steering = (event.state - 128) / 128
#                 twist_msg.angular.z = (event.state - 128) / 128
#                 #print("Lenkung: {0}".format(twist_msg.angular.z))
        
                    
#         # for event in events:
#         #     print(event.ev_type, event.code, event.state)


#         #pygame.event.pump()
        
#         #Obtain gamepad values
#         #throttle = j.get_axis(1) #Left thumbstick Y
#         #steering = j.get_axis(2) #Right thumbstick X
#         #print("Throttle:", throttle)
#         #print("Steering:", steering)

#         # #Publish gamepad values
#         #pub_throttle.publish(throttle)
#         #pub_steering.publish(steering)
#         pub_ackerman.publish(twist_msg)
        
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         teleop_gamepad()
#     except rospy.ROSInterruptException:
#         pass


#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# class teleop_gamepad():

    # def init(self):
    #     rospy.init_node("teleop", anonymous=True)
    #     self.throttle_axis = 4 # right analog stick
    #     self.steering_axis = 0 # left analog stick, left == 1.0, right == -1.0
    #     self.deadman_switch = 4 # L1 has to be pressed at all times       
    #     rospy.Subscriber("gamepad_listener", _Joy, self.callback_teleop)
    #     self.throttle_pub = rospy.Publisher("throttle", Float32, queue_size=1)
    #     self.steering_pub = rospy.Publisher("steering", Float32, queue_size=1)
    #     self.rate = rospy.Rate(1000)


def callback_teleop(data):

    print("I heard sth...")

    throttle_axes = 1 # left analog stick, up and down
    steering_axes = 2 # right analog stick, left == 1.0, right == -1.0
    deadman_switch = 4 # L1 has to be pressed at all times

    if data.buttons[deadman_switch] == 1: 
         
        throttle_pub = rospy.Publisher("throttle", Float32, queue_size=1)
        steering_pub = rospy.Publisher("steering", Float32, queue_size=1) 
        throttle_pub.publish(data.axes[throttle_axes] * -1)
        steering_pub.publish(data.axes[steering_axes] * -1)
        # cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # cmd_vel = Twist()
        # cmd_vel.linear.x = data.axes[throttle_axes]
        # cmd_vel.angular.z = data.axes[steering_axes] 
        # cmd_vel_pub.publish(cmd_vel)
        

def teleop():
    rospy.init_node("teleop", anonymous=True)
    throttle_axis = 4 # right analog stick
    steering_axis = 0 # left analog stick, left == 1.0, right == -1.0
    deadman_switch = 4 # L1 has to be pressed at all times       
    rospy.Subscriber("joy", Joy, callback_teleop)
    
    throttle_pub = rospy.Publisher("throttle", Float32, queue_size=1)
    steering_pub = rospy.Publisher("steering", Float32, queue_size=1)  
    
    rospy.spin()
    
    # while not rospy.is_shutdown():

    #     rate = rospy.Rate(1000)
    #     rate.sleep()
        # rospy.sleep(0.05)


if __name__ == '__main__':
    print("Running teleop.py")
    teleop()
