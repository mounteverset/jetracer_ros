#!/usr/bin/env python3

# import rospy
# import pygame
# import time
# from std_msgs.msg import Float32

# #Initialize pygame and gamepad
# pygame.init()
# j = pygame.joystick.Joystick(0)
# j.init()
# print ('Initialized Joystick : %s' % j.get_name())

# def teleop_gamepad():
#     #Setup topics publishing and nodes
#     pub_throttle = rospy.Publisher('throttle', Float32, queue_size=8)
#     pub_steering = rospy.Publisher('steering', Float32, queue_size=8)
#     rospy.init_node('teleop_gamepad', anonymous=True)
#     rate = rospy.Rate(10) # 10hz

#     while not rospy.is_shutdown():
#         pygame.event.pump()
        
#         #Obtain gamepad values
#         throttle = j.get_axis(1) #Left thumbstick Y
#         steering = j.get_axis(2) #Right thumbstick X
#         print("Throttle:", throttle)
#         print("Steering:", steering)

#         #Pubblish gamepad values
#         pub_throttle.publish(throttle)
#         pub_steering.publish(steering)
        
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         teleop_gamepad()
#     except rospy.ROSInterruptException:
#         pass



import rospy
#import pygame
import time
from inputs import devices, get_gamepad, GamePad
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, TwistStamped


gamepadd = devices.gamepads[0]


def teleop_gamepad():

    #Setup topics publishing and nodes
    rospy.init_node('teleop_gamepad', anonymous=True)

    pub_ackerman = rospy.Publisher('ackerman_control/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(1000) # 10hz


    twist_msg = Twist()
    save_event = 0

    while not rospy.is_shutdown():


        gamepad = gamepadd.read()
    
        for event in gamepad:
            print(event.ev_type)
            print("!!!!!!!!!!!!!!!!!")
            print(event.code)
            print("!!!!!!!!!!!!!!!!!")
            print(event.state)
            
            if (event.code == "ABS_Y"):
                #throttle = (event.state - 128) / 128
                save_event = -10*((event.state - 128) / 128)
                twist_msg.linear.x = save_event
                #print ("Gas: {0}".format(twist_msg.linear.x))
                
            elif (event.code == "ABS_Z"):
                #steering = (event.state - 128) / 128
                twist_msg.angular.z = (event.state - 128) / 128
                #print("Lenkung: {0}".format(twist_msg.angular.z))
        
                    
        # for event in events:
        #     print(event.ev_type, event.code, event.state)


        #pygame.event.pump()
        
        #Obtain gamepad values
        #throttle = j.get_axis(1) #Left thumbstick Y
        #steering = j.get_axis(2) #Right thumbstick X
        #print("Throttle:", throttle)
        #print("Steering:", steering)

        # #Publish gamepad values
        #pub_throttle.publish(throttle)
        #pub_steering.publish(steering)
        pub_ackerman.publish(twist_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        teleop_gamepad()
    except rospy.ROSInterruptException:
        pass