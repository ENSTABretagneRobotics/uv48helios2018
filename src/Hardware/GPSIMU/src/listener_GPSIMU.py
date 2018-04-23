#!/usr/bin/env python
import rospy

# from std_msgs.msg import String
from sbg_driver.msg import SbgGpsPos
from sbg_driver.msg import SbgEkfEuler
from sbg_driver.msg import SbgEkfNav



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.angle.x)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.angle.y)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.angle.z)

def listener():
    rospy.init_node('listener', anonymous=True)
    # rospy.Subscriber("chatter", String, callback)

    rospy.Subscriber("ekf_euler", SbgEkfEuler, callback)
    # rospy.Subscriber("gps_pos", SbgGpsPos, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
