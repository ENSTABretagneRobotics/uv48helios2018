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

def callback_ekfeuler(data):
    rospy.loginfo(rospy.get_caller_id() + "IMU : I heard %s", data.angle.x)
    rospy.loginfo(rospy.get_caller_id() + "IMU : I heard %s", data.angle.y)
    rospy.loginfo(rospy.get_caller_id() + "IMU : I heard %s", data.angle.z)
    IMU.angular.x = data.angle.x
    IMU.angular.y = data.angle.y
    IMU.angular.z = data.angle.z

def callback_gpspos(data):
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.x)
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.y)
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.z)
    GPS.position.x, GPS.position.y = conv_latlong_lamb93(data.position.x, data.position.y)
    GPS.position.z = data.position.z

def callback_ekfnav(data):
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.x)
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.y)
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.z)


# def chat_GPS(data):
#     rospy.loginfo("CHAT GPS\n")
#
# def chat_IMU(data):
#     rospy.loginfo("CHAT IMU\n")
#

#def talker():



rospy.init_node('listener') #, anonymous=True)
rospy.Subscriber("ekf_euler", SbgEkfEuler, callback_ekfeuler)
rospy.Subscriber("gps_pos", SbgGpsPos, callback_gpspos)

rospy.loginfo("CHAT GPS\n")
GPS = rospy.Publisher('Pose_GPS_cart', Pose, 10)
rospy.loginfo("CHAT IMU\n")
IMU = rospy.Publisher('Twist_GPS_cart', Twist, 10)


IMU.publish()
GPS.publish()
r = rospy.Rate(10)
