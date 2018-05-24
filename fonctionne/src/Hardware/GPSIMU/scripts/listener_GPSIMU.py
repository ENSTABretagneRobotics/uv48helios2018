#!/usr/bin/env python
import rospy
from math import cos,sin,log, exp,pi

# from std_msgs.msg import String
from sbg_driver.msg import SbgGpsPos
from sbg_driver.msg import SbgEkfEuler
from sbg_driver.msg import SbgEkfNav
from geometry_msgs.msg import Pose, Twist

n = 0.7256077650
C = 11754255.426
Xs = 700000.000
Ys = 12655612.05
e2 = 0.08248325676

def degminsec_to_deg(deg,minu,sec):
    return deg + minu/60 + sec/3600

def deg_to_rad(deg):
    return deg*pi/180

# lam0 = deg_to_rad(degminsec_to_deg(2,20,14.025)) # A verifier
lam0 = deg_to_rad(degminsec_to_deg(3,0,0))

def conv_latlong_lamb93(lat, longi):
    lam = lat
    phi = longi
    epsi = 0.5*log((1+sin(phi))/(1-sin(phi))) - 0.5*e2*log((1+e2*sin(phi))/(1-e2*sin(phi)))
    gamma = n*(lam - lam0)
    R = C*exp(-n*epsi)
    X = Xs + R*sin(gamma)
    Y = Ys -R*cos(gamma)
    return X,Y

def callback_IMU(data):
    rospy.loginfo(rospy.get_caller_id() + "IMU : I heard %s", data.angle.x)
    rospy.loginfo(rospy.get_caller_id() + "IMU : I heard %s", data.angle.y)
    rospy.loginfo(rospy.get_caller_id() + "IMU : I heard %s", data.angle.z)
    phi, theta, psi = data

def callback_GPS(data):
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.x)
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.y)
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.z)
    msg_GPS = Pose()
    msg_GPS.position.x = data.position.x
    msg_GPS.position.y = data.position.y
    msg_GPS.position.z = data.position.z

def chat_GPS(data):
    rospy.loginfo("CHAT GPS\n")

def chat_IMU(data):
    rospy.loginfo("CHAT IMU\n")
    IMU.publish()

def talker():



rospy.init_node('listener') #, anonymous=True)
rospy.Subscriber("ekf_euler", SbgEkfEuler, callback_IMU)
rospy.Subscriber("gps_pos", SbgGpsPos, callback_GPS)

GPS = rospy.publisher('Pose_GPS_cart', Pose, chat_GPS)
IMU = rospy.publisher('Twist_GPS_cart', Twist, chat_IMU)

r = rospy.Rate(10)
x,y = 0,0

msg_IMU = SbgEkfEuler()
