#!/usr/bin/env python3

from numpy import mean,pi,cos,sin,sqrt,tan,arctan,arctan2,tanh,arcsin,exp,dot,array,log,inf, eye, zeros, ones, inf,size,arange,reshape,concatenate,vstack,hstack,diag,median,sign,sum,meshgrid,cross,linspace,append,round
from numpy.linalg import inv, det, norm, eig
import rospy
from std_msgs import Float64MultiArray
from geometry_msgs import Pose2D, PoseArray, 

K = 1 

a= array([[0, 0]]).T
b= array([[10, 10]]).T

x = array([[-9],[6],[pi]]) # X = [x,y,theta]
erreur_angle = 0

v =1



def traj(liste):    # la liste des points de la trajectoire (x,y)
    global x
    global a
    global b

    
    m = x[0:2]
    phi = arctan2(b[1, 0]-a[1,0]  ,  b[0,0]-a[0,0] )
    m = x[0:2]
    mat = array([[(b-a)[0, 0],(m-a)[0, 0]],[(b-a)[1, 0],(m-a)[1, 0]]])
    e = det(mat)/norm(b-a)
    thetabar = phi - arctan(e)
    erreur_angle = thetabar - x[2][0]
    return erreur_angle        


def callback1(data):
    global liste_position
    
    xa = data.poses[0].position.x
    ya = data.poses[0].position.y
    xb = data.poses[1].position.x
    yb = data.poses[1].position.y
    
    a = [xa, ya]
    b = [xb, yb]
    
def callback2(data):
    global x
    global y
    global theta
    
    x = data.position.x
    y = data.position.y
    theta = data.orientation.z

    
def sortie_orientation_vitesse():
    
    rospy.Subscriber("/Path_Direction", PoseArray, callback1)
    rospy.Subscriber("/Position", Pose, callback2)

    pub_commande = rospy.Publisher('/Twist_commande', Float64MultiArray, queue_size=10)

    rospy.init_node('Regulateur_ligne', anonymous=True)    
    rate = rospy.Rate(20) # 10hz   
     
    msg_commande = Float64MultiArray()

    while not rospy.is_shutdown():
        erreur_angle = traj(liste_position)       
        msg_commande.data.append(erreur_angle)
        msg_commande.data.append(v)
        pub_commande.publish(msg_commande)
        
        rate.sleep()
    
#==============================================================================
# Main
#==============================================================================

if __name__ == '__main__':
    
    try:
        sortie_orientation_vitesse()
    except rospy.ROSInterruptException:
        pass                
