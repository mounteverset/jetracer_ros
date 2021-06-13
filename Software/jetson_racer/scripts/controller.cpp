#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream> 
#include <string.h> 
  
using namespace std; 

// Topic messages callback
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{

   ROS_INFO("[Listener] I heard: [%s]\n", msg->data.c_str());

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

   geometry_msgs::Twist vel;
    
   vel.linear.x = double(1.0);
   vel.angular.z = double(1.0);

  
 
   pub.publish(vel);
   //ROS_INFO_STREAM("Filtered velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lf_controller");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/lineSensor", 1000, chatterCallback);
    ros::spin();

   return 0;
}