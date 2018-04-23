#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include <string>
#include <cmath>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int64.h"
#include "../include/asservissement/maestro.h"
#include "../include/asservissement/state.h"

using namespace Eigen;
using namespace std;
double u_gauche, u_droite
double err_angle, vitesse;
double k = 4000.0;
State state;

void chatCallback(const std_msgs::Float64MultiArray msg){
    err_angle = msg.data[0];
    vitesse   = msg.data[1];
}


//float64Array: vitesses angulaires

void differentiel (){
    
    Matrix2d M;
    M << 1/2.0 , 1/2.0, -1/2.0 , 1/2.0;
    Matrix2d M_inv = M.inverse();
    Vector2d A (0.5,arctan(tan(err_angle/2)));
    Vector2d u = 1/R *( M_inv * A);
    //u entre -1 et 1
    u_gauche = u[0];
    u_droite = u[1];

    //Satuation vitesse, haute
    double a = max(u_gauche, u_droite, 1);
    double b = min(u_gauche, u_droite, -1);

    //1er cas: grosse difference-->on trourne vite et sur place
    if (a-b > 2){
        if (u_gauche > u_droite){
            u_gauche = 1;
            u_droite = -1;
        }
        else{
            u_gauche = -1;
            u_droite = 1;
        }

    }

    //2eme cas Satuation vitesse, haute
    elif (a>=1){
        double diff = a-1;
        u_gauche -= diff;
        u_droite -= diff;
        if (u_gauche<-1){
            u_gauche=-1;
        }
        if (u_droite<-1){
            u_droite=-1;
    }
    //3 eme cas Satuation vitesse, basse
    elif (a<=-1){
        double diff = -a-1;
        u_gauche += diff;
        u_droite += diff;

        if (u_gauche>1){
            u_gauche=1;
        }

        if (u_droite>1){
            u_droite=1;
        }
    }

    //passage entre 0 et 8000
    u_gauche = (1 + u_gauche) * k;
    u_droite = (1 + u_droite) * k;
    }

void init_servo(int fd)
{
    maestroSetTarget(fd, 0, 4000.0);
    maestroSetTarget(fd, 1,4000.0);
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

    n.param<string>("Path", path, "/dev/ttyACM0");
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

//    ros::ServiceServer state_srv = n.advertiseService("pololu_state", stateCallback);

    while(ros::ok())
    {
        //calcul de u
        differentiel()

        if(true)  //Condition de fonctionnement (diagramme d'état)
        {
            left_mot 	= u_gauche;
            right_mot = u_droite;
        }

        else {
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
