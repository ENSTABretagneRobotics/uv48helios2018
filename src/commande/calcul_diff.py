# -*- coding: utf-8 -*-
"""
Created on Tue Mar  6 20:56:04 2018

@author: Juliette
"""

import rospy
from std_msgs.msg import int
from geometry_msgs import Pose, Twist

mot_g = rospy.Publisher("moteur_G",int, 1000)
msg1 = std_msgs.int()
mot_d = rospy.Publisher("moteur_D",int, 1000)
msg2 = std_msgs.int()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listen_commande():
    rospy.init_node("listen_commande", anonymous = True)
    rospy.Suscriber("commande", Twist, callback)
    ros.spin()

def listen_manette():
    rospy.init_node("listen_manette", anonymous = True)
    rospy.Suscriber("manette", Twist, callback)
    ros.spin()

msg1.data = u[0]
mot_g.Publish(msg1)
msg2.data = u[1]
mot_d.Publish(msg2)