#include <time.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <irobot_tracker/trackerDebug.h>
#include "PlainBase.h"

#define SCALE 10
ros::Publisher myPlain_pub;

geometry_msgs::Pose est;
geometry_msgs::Pose mes;
geometry_msgs::Vector3 mySpeed;
geometry_msgs::Vector3 myAccelerate;
PlainBase myPlain;

using namespace std;

void debug_rviz(const irobot_tracker::trackerDebug::ConstPtr& msg)
{
	ROS_INFO("herer");
	est.position.x = msg -> filter_pos.x/100.0f;
	est.position.y = msg -> filter_pos.y/100.0f;
	est.position.z = msg -> filter_pos.z/100.0f;
	est.orientation.x= msg -> filter_ort.x;
	est.orientation.y= msg -> filter_ort.y;
	est.orientation.z= msg -> filter_ort.z;
	est.orientation.w= msg -> filter_ort.w;

	mes.position.x = msg -> vision_pos.x/100.0f;
	mes.position.y = msg -> vision_pos.y/100.0f;
	mes.position.z = msg -> vision_pos.z/100.0f;
	mes.orientation.x=msg -> imu_ort.x;
	mes.orientation.y=msg -> imu_ort.y;
	mes.orientation.z=msg -> imu_ort.z;
	mes.orientation.w=msg -> imu_ort.w;

	mySpeed.x = msg -> filter_speed.x/100;
	mySpeed.y = msg -> filter_speed.y/100;
	mySpeed.z = msg -> filter_speed.z/100;

	myAccelerate.x = msg -> imu_linear_a.x/100;
	myAccelerate.y = msg -> imu_linear_a.y/100;
	myAccelerate.z = msg -> imu_linear_a.z/100;
	
	myPlain.update(est,mes,mySpeed,myAccelerate);
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "Tracker_Debug2222");
	ros::NodeHandle nh;
	myPlain_pub = nh.advertise<visualization_msgs::Marker>("myPlain", 100);
	myPlain.setPub(myPlain_pub);
	ros::Subscriber sub = nh.subscribe("/tracker_debug", 100, debug_rviz);
	ros::spin();
	return 0;
}
