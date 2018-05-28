#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Bool_is_OK");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::Bool>("Bool_is_OK", 1);
  //ros::Publisher f = nh.advertise<std_msgs::Float64>("f", 1024);

  ros::Rate loop_rate(20);
  float fl = 0.0;
  while (ros::ok())
  {
    fl = fl+1.0;
    //std_msgs::Float64 m2;
    std_msgs::Bool msg;
    msg.data = 1;
    //m2.data = fl;

    chatter_pub.publish(msg);
    //f.publish(m2);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
