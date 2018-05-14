#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from pyibex import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from geometry_msgs.smg import Twist


from pyibex import *


def chat_GPS(msg):
    global x_Bat = msg.x
    global y_Bat = msg.y
    global theta_Bat = msg.theta

    
def chat_IMU(msg):
    global x_Bat_dot = msg.linear.x
    global y_Bat_dot = msg.linear.y
    global theta_Bat_dot = msg.angular.z



def talker():
    pose_vect = rospy.Publisher('Pose_veect_X', Pose2D, queue_size=10)
    twist_vect = rospy.Publisher('Twist_veect_X', Twist, queue_size=10)
    
    GPS = rospy.Subscriber('Pose_GPS_cart', Pose2D, chat_GPS)
    IMU = rospy.Subscriber('Pose_GPS_cart', Twist, chat_IMU)
    
    msg_GPS = Pose2D()
    msg_IMU = Twist()

    rospy.init_node('loclocloc', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        
                
        pose_vect.publish(msg1)
        twist_vect.publish(msg2)

        rate.sleep()
   
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
