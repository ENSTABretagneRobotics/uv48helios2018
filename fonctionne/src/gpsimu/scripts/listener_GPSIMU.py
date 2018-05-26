#!/usr/bin/env python
import rospy
from math import cos,sin,log, exp,pi

# from std_msgs.msg import String
from sbg_driver.msg import SbgGpsPos
from sbg_driver.msg import SbgEkfEuler
from sbg_driver.msg import SbgEkfNav
from sbg_driver.msg import SbgImuData
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_euler

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
    q = quaternion_from_euler(data.angle.x, data.angle.y, data.angle.z)
    msg_Pose.orientation = q

def callback_GPS(data):
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.x)
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.y)
    rospy.loginfo(rospy.get_caller_id() + "GPS : I heard %s", data.position.z)
    x, y = conv_latlong_lamb93(data.position.x, data.position.y)
    msg_Pose.position.x = x
    msg_Pose.position.y = y
    msg_Pose.position.z = data.position.z

def callback_vel(data):
    rospy.loginfo(rospy.get_caller_id() + "vel : I heard %s", data.velocity.x)
    rospy.loginfo(rospy.get_caller_id() + "vel : I heard %s", data.velocity.y)
    rospy.loginfo(rospy.get_caller_id() + "vel : I heard %s", data.velocity.z)
    msg_Twist.linear = data.velocity

def callback_ang_vel(data):
    rospy.loginfo(rospy.get_caller_id() + "vel : I heard %s", data.gyro.x)
    rospy.loginfo(rospy.get_caller_id() + "vel : I heard %s", data.gyro.y)
    rospy.loginfo(rospy.get_caller_id() + "vel : I heard %s", data.gyro.z)
    msg_Twist.angular = data.gyro

rospy.init_node('listener') #, anonymous=True)
GPS = rospy.Publisher('Pose_GPS_cart', Pose, queue_size = 10)
IMU = rospy.Publisher('Twist_GPS_cart', Twist, queue_size = 10)

msg_Pose = Pose()
msg_Twist = Twist()

rospy.Subscriber("ekf_euler", SbgEkfEuler, callback_IMU)
rospy.Subscriber("gps_pos", SbgGpsPos, callback_GPS)
rospy.Subscriber("ekf_nav", SbgEkfNav, callback_vel)
rospy.Subscriber("imu_data", SbgImuData, callback_ang_vel)


r = rospy.Rate(10)
x,y = 0,0


while not rospy.is_shutdown():

    GPS.publish(msg_Pose)
    IMU.publish(msg_Twist)
    r.sleep()
