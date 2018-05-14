#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hector_std_msgs/Float64Array.h"


int main(int argc, char ∗∗argv){
ros::init(argc, argv, "pololu");

ros::NodeHandle n;
ros::Publisher chatter_pub = n.advertise<hector_std_msgs::Float64Array>("vitesse", 1000);
ros::Rate loop_rate(10);

while (ros::ok()){

hector_std_msgs::Float64Array
double t = ros::Time::now().toSec();
msg.data = [0.1*t,0.1*t];

chatter_pub.publish(msg);
ros::spinOnce();

loop_rate.sleep();
}
return 0;
}