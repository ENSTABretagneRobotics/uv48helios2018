#!/usr/bin/env python
import rospy
from pyibex import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from vibes import *

def chat_GPS(msg):

    #global pos_Bat_GPS
    #a = Interval(0.1,0.3)
    #rospy.loginfo("%s",msg.x)
    pos_Bat_GPS = IntervalVector([[msg.x - 3.0, msg.x + 3.0,][msg.y - 3.0, msg.y + 3.0]])
    reduction(pos_Bat_GPS)

def reduction(pos_Bat_GPS):
    if (pos_Bat & pos_Bat_GPS == 0):
        pos_Bat = pos_Bat_GPS
    else:
        pos_Bat &= pos_Bat_GPS


def chat_IMU(msg):
    k = 10.0
    k2 = 0.3
    pos_Bat_dot = IntervalVector([[msg.linear.x-k,msg.linear.x + k],[msg.linear.y-k,msg.linear.y + k]])
    theta_Bat_dot = Interval(msg.angular.z - k2,msg.angular.z + k2)

def drawBox(x, color="b"):
    vibes.drawBox(x[0].lb(), x[0].ub(), x[1].lb(), x[1].ub(), color)



def talker():
    # Initialisation des Variables
    global pos_Bat, pos_Bat_dot, theta_Bat_dot
    pos_Bat = IntervalVector([[0,0],[0,0]])
    pos_Bat_dot = IntervalVector([[0,0],[0,0]])
    theta_Bat_dot = Interval(0,0)
    dt = 0.1

    # Publishers
    pose_vect = rospy.Publisher('Pose_veect_X', Pose2D, queue_size=10)
    twist_vect = rospy.Publisher('Twist_veect_X', Twist, queue_size=10)
    # Subscribers
    GPS = rospy.Subscriber('Pose_GPS_cart', Pose2D, chat_GPS)
    IMU = rospy.Subscriber('Twist_GPS_cart', Twist, chat_IMU)
    # Creation des messages
    msg_coord = Pose2D()
    msg_coord_dot = Twist()
    # Initialisation du node
    rospy.init_node('Fusion_capteur', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # Calcul des coordonnees du bateau
        pos_Bat[0] += pos_Bat_dot[0] * cos(theta_Bat_dot) * dt
        pos_Bat[1] += pos_Bat_dot[1] * sin(theta_Bat_dot) * dt
        drawBox(pos_Bat)
        # Remplissage des messages
        msg_coord.x = pos_Bat[0]
        msg_coord.y = pos_Bat[1]
        msg_coord_dot.linear.x = pos_Bat_dot[0]
        msg_coord_dot.linear.y = pos_Bat_dot[1]
        msg_coord_dot.angular.z = theta_Bat_dot
        # Publication des messages
        pose_vect.publish(msg_coord)
        twist_vect.publish(msg_coord_dot)

        rate.sleep()

if __name__ == '__main__':
    try:

        vibes.beginDrawing()
        vibes.newFigure("Test")
        vibes.setFigureProperties({'x':0, 'y':0, 'width':600, 'height':600})
        vibes.axisLimits(-3,30,-4,30)

        talker()
        vibes.endDrawing()
    except rospy.ROSInterruptException:
        pass
