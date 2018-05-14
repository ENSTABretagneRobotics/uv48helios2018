#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"
#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/Image.h"
//#include <string>

struct X{
  double x;
  double y;
  double theta;
  double xp;
  double yp;
};

typedef struct X X_state;

double bat_pourcentage;
double moteurG;
double moteurD;
std_msgs::String etat_fsm;
X_state X;
long int numero;
sensor_msgs::Image webcam;


void batterieCallback(const std_msgs::Float64::ConstPtr& msg){
  bat_pourcentage = msg->data;
}

void moteurGCallback(const std_msgs::Float64::ConstPtr& msg){
  moteurG = msg->data;
}

void moteurDCallback(const std_msgs::Float64::ConstPtr& msg){
  moteurD = msg->data;
}

void etatCallback(const std_msgs::String::ConstPtr& msg){
  etat_fsm.data = msg->data;
}

void vectorCallback(const geometry_msgs::Pose::ConstPtr& msg){
  X.x = msg->position.x;
  X.y = msg->position.y;
  X.theta = tf::getYaw(msg->orientation);
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
  X.xp = msg->linear.x;
  X.yp = msg->linear.y;
}

void numeroCallback(const std_msgs::Int64::ConstPtr& msg){
  numero = msg->data;
}

void webcamCallback(const sensor_msgs::Image::ConstPtr& msg){
  webcam.data         =   msg->data;
  webcam.header       =   msg->header;
  webcam.height       =   msg->height;
  webcam.width        =   msg->width;
  webcam.encoding     =   msg->encoding;
  webcam.is_bigendian =   msg->is_bigendian;
  webcam.step         =   msg->step;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Reception_Data");
  ros::NodeHandle nh;

  ros::Subscriber bat   = nh.subscribe("Tension_batteries", 1000, batterieCallback);
  ros::Subscriber motg  = nh.subscribe("Int_puis_mot_G", 1000, moteurGCallback);
  ros::Subscriber motd  = nh.subscribe("Int_puis_mot_D", 1000, moteurDCallback);
  ros::Subscriber etat  = nh.subscribe("String_Etat", 1000, etatCallback);
  ros::Subscriber vectx = nh.subscribe("Pose_vect_X", 1000, vectorCallback);
  ros::Subscriber twist = nh.subscribe("Twist_vect_X",1000,twistCallback);
  //ros::Subscriber Gps   = nh.subscribe("gps", 1000, gpsCallback);
  ros::Subscriber num   = nh.subscribe("numero", 1000, numeroCallback);//int64
  ros::Subscriber cam   = nh.subscribe("Image_compressed_Webcam",1000,webcamCallback);
  bool connected=true;

  ros::Rate loop_rate(20);
  while (true){
    connected = ros::ok();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
