#include "ros/ros.h"
#include <iostream>
#include <string>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int64.h"
#include "BathyBoatNav/new_state.h"
#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "maestro.h"
#include "../include/state.h"

using namespace std;
double u_throttle, u_yaw;

State state;

void chatCallback(const hector_std_msgs::Float64Array msg)
//float64Array: vitesses angulaires
{
    u_gauche 	= msg->data[0];
    u_droite    = msg->data[1];
}

void init_servo(int fd)
{
    maestroSetTarget(fd, 0, 4000);
    maestroSetTarget(fd, 1, 4000);
    sleep(2);
}

int main(int argc, char *argv [])
{
    int fd;
    string path;
    string channel;
    int gap;

    bool isSimulation;

    double left_mot, right_mot;
    left_mot = 0;
    right_mot = 0;

    // Ros init

    ros::init(argc, argv, "pololu");
    ros::NodeHandle n;

    ros::Rate loop_rate(25);

    // Initials parameters

    n.param<string>("Path", path, "/dev/pololu_servo_serial");
    n.param<string>("Cons_channel", channel, "cons_boat");
    n.param<int>("Turn_gap", gap, 1000);

    // Connection to Maestro

    if( (fd = maestroConnect(path.c_str())) == -1 )
    {
        ROS_INFO("Pololu not found. Simulation mode.");
        isSimulation = true;
    } else {
        ROS_INFO("Pololu connected");
        init_servo(fd);
        isSimulation = false;
    }

    // Subscribe msgs

    ros::Subscriber cons_sub = n.subscribe("vitesse", 1000, chatCallback);

    // Publisher

    ros::Publisher left_mot_pub = n.advertise<std_msgs::Int64>("left_mot", 1000);
    std_msgs::Int64 left_mot_msgs;

    ros::Publisher right_mot_pub = n.advertise<std_msgs::Int64>("right_mot", 1000);
    std_msgs::Int64 right_mot_msgs;

    // State service

    ros::ServiceServer state_srv = n.advertiseService("pololu_state", stateCallback);

    while(ros::ok())
    {
        if(state == RUNNING)
        {
            left_mot 	= u_gauche;
            right_mot 	= u_droite;
        } else {
            left_mot 	= 4000.0;
            right_mot 	= 4000.0;
        }

        left_mot_msgs.data = left_mot;
        left_mot_pub.publish(left_mot_msgs);

        right_mot_msgs.data = right_mot;
        right_mot_pub.publish(right_mot_msgs);

        if(!isSimulation)
        {
            maestroSetTarget(fd, 1, left_mot);
            maestroSetTarget(fd, 0, right_mot);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    exit(0);
}