#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Header.h"
#include <vector>

int main(int argc,char** argv){
    ros::init(argc, argv, "Path_test");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<nav_msgs::Path>("WayPoint", 1000);
    ros::Rate loop_rate(20);


    while(ros::ok()){
        nav_msgs::Path msg;
        geometry_msgs::PoseStamped a,b;
        a.pose.position.x = 53;
        a.pose.position.y = 1;

        b.pose.position.x = 5;
        b.pose.position.y = 4;
        std::vector<geometry_msgs::PoseStamped> envoi;
        envoi.push_back(a);
        envoi.push_back(b);
        msg.poses.resize(envoi.size());
        for(unsigned int i=0; i < envoi.size(); i++){
            msg.poses[i] = envoi[i];
        }
        chatter_pub.publish(msg);
        loop_rate.sleep();
    }



    return 0;
}
