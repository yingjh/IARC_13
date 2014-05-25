#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

//void sendposition(visualization_msgs::Marker & mrk);
void sendposition(boost::shared_ptr<visualization_msgs::Marker const>& msg);

int main(int argc, char** argv) {

	ros::init(argc, argv, "RvizL");

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("tracker_position", 1000, sendposition);
	ros::spin();
 
     	return 0;
}

void sendposition(boost::shared_ptr<visualization_msgs::Marker const>& msg)
{
	ROS_INFO("Get_type: x=%d",msg->type);
}
