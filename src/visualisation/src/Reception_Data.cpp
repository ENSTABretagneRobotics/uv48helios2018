#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/tf.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/CompressedImage.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <iostream>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <fcntl.h>


#define BUFFSIZE 1048576 //2**20
#define TAILLE 256

typedef struct Id_Co Id_Co;
typedef struct sockaddr_in sockaddr_in;
typedef struct sockaddr sockaddr;


struct Id_Co{
  int* sock_ptr;
  int nb;
};


namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}


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
//sensor_msgs::CompressedImage webcam;
cv_bridge::CvImageConstPtr cv_ptr;

void *shell(void*info){
}





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
void webcamCallback(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char **argv)
{
 	int port = 8080;
  int socket_desc, client_sock,c,read_size;
  char buffer_recu[BUFFSIZE] , buffer_envoi[BUFFSIZE];
  Id_Co info;
	socket_desc = socket(AF_INET,SOCK_STREAM,0);
	struct sockaddr_in server,client;


  server.sin_addr.s_addr = inet_addr("127.0.0.1");
  server.sin_family = AF_INET;
  server.sin_port = htons(port);



  Mat image;

  ros::init(argc, argv, "Reception_Data");
  ros::NodeHandle nh;

  ros::Subscriber bat    = nh.subscribe("Tension_batteries", 1000, batterieCallback);
  ros::Subscriber motg   = nh.subscribe("Int_puis_mot_G", 1000, moteurGCallback);
  ros::Subscriber motd   = nh.subscribe("Int_puis_mot_D", 1000, moteurDCallback);
  ros::Subscriber vectx  = nh.subscribe("Pose_vect_X", 1000, vectorCallback);
  ros::Subscriber twist  = nh.subscribe("Twist_vect_X",1000,twistCallback);
  ros::Subscriber num    = nh.subscribe("numero", 1000, numeroCallback);//int64
  ros::Subscriber cam    = nh.subscribe("usb_cam/image_raw/compressed",1000,webcamCallback);

  ros::Publisher etat    = nh.advertise<std_msgs::String>("String_Etat", 1000);
  ros::Publisher send_wp = nh.advertise<nav_msgs::Path>("Path_WayPoints",1000);
  bool connected=true;

   

  ros::Rate loop_rate(20);
  c = sizeof(sockaddr_in);

  while(client_sock = accept(socket_desc,(struct sockaddr*)&client,(socklen_t*)&c)){
    connected = ros::ok();
    pthread_t new_connected;
    if(pthread_create(&new_connected,NULL,shell,(void*)&info)<0){
      perror("error starting new user shell thread");
      return EXIT_FAILURE;
    }

    memset(buffer_envoi,0,TAILLE);
    
    write(client_sock,buffer_envoi,strlen(buffer_envoi));

    image = cv_ptr->image;

    write(client_sock,buffer_envoi,strlen(buffer_envoi));



    nav_msgs::Path msg;
    send_wp.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
