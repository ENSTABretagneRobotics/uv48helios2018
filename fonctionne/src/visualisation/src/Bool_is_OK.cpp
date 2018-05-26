#include "ros/ros.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Bool_is_OK");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::Bool>("Bool_is_OK", 1);

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    std_msgs::Bool msg;
    msg.data = 1;

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
