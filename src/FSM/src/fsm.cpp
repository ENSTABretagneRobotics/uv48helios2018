#include "ros/ros.h"
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Path.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"

#include "tf/tf.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define ERR_DISTANCE 5


float x_helios, y_helios;
float pi = 3.1415926535;
std::vector<geometry_msgs::PoseStamped> msg_waypoints;
std::array<std::array<float, 2>, 2> ab;// = { {ax, bx},{ay, by} };
std::array<std::array<float, 2>, 1> helios;
std::array<std::array<float, 2>, 1> d12, d2;


void commandeCallback(const nav_msgs::Path::ConstPtr& msg_path){
 msg_waypoints = msg_path->poses;
  //ROS_INFO("waypoints received from target");
}

void vectXCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_helios){
  x_helios = msg_helios->pose.position.x;
  y_helios = msg_helios->pose.position.y;
  helios[0][0] = x_helios;
  helios[1][0] = y_helios;
}

int main(int argc, char **argv){
    std::array<std::array<float, 2>, 2> ab;// = { {ax, bx},{ay, by} };
    std::array<std::array<float, 2>, 1> helios;

    float x = 0;
    float y = 0;
    int lenght_wpts;
    float theta, command;
    float e;
    int i = 0;

    ros::init(argc, argv, "FSM");
    ros::NodeHandle n;

    //Ecoute sur
    //ros::Subscriber vectX_sub = n.subscribe("vect_X", 1, vectXCallback);
    //ros::Subscriber commande_sub = n.subscribe("WayPoint", 1, commandeCallback);

    //Publie sur
    ros::Publisher etats_pub = n.advertise<std_msgs::String>("Etats", 1);
    ros::Publisher direction_pub = n.advertise<geometry_msgs::PoseArray>("Pose_Direction", 1);
    ros::Publisher position_pub = n.advertise<std_msgs::Float64>("Path_Position", 1);
    ros::Publisher num_wpt_pub = n.advertise<std_msgs::Int64>("Numero", 1);
    ros::Rate loop_rate(20);

    while(ros::ok()){
        lenght_wpts = msg_waypoints.size();

/*
        ab[0,0] = msg_waypoints[i].pose.position.x;
        ab[1,0] = msg_waypoints[i].pose.position.y;
        ab[0,1] = msg_waypoints[i+1].pose.position.x;
        ab[1,1] = msg_waypoints[i+1].pose.position.y;
*/
        ab[0][0] = 5;
        ab[1][0] = 3;
        ab[0][1] = 10;
        ab[1][1] = 3;
        helios[0][0] = 7;
        helios[1][0] = 6;

        geometry_msgs::PoseArray msg_ab;
        msg_ab.header.stamp = ros::Time::now();
        msg_ab.poses[0].position.x = ab[0][0];
        msg_ab.poses[0].position.y = ab[1][0];
        msg_ab.poses[1].position.x = ab[0][1];
        msg_ab.poses[1].position.y = ab[1][1];

        e = (ab[0][1] - ab[0][0])*(helios[1][0] - ab[1][0]) -
            (ab[1][1] - ab[1][0])*(helios[0][0] - ab[0][0]);

        //passage aux pts suivants
        if (e < ERR_DISTANCE){
          i++;
        }
        //fini de lire tous les wayoints
        /*
        if (i > lenght_wpts-1) {
          x= 0;
        } // fin if
        */

        std_msgs::String msg_etat;
        msg_etat.data = "none";
        std_msgs::Int64 msg_num_wpts;
        msg_num_wpts.data = i++;

        //Publication des messages
        direction_pub.publish(msg_ab);
        etats_pub.publish(msg_etat);
        num_wpt_pub.publish(msg_num_wpts);
        ros::spinOnce();
        loop_rate.sleep();

    } // fin while
    return 0;
}
