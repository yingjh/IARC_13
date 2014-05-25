#include "ros/ros.h"
#include "opencvmarkdetect/Xyz.h"
#include <visualization_msgs/Marker.h>

opencvmarkdetect::Xyz xyz;
//name for RViz visualization_marker
ros::Publisher marker_pub;
visualization_msgs::Marker trackerpoints;


void xyzCallback(const opencvmarkdetect::Xyz& msg)
{
	ROS_INFO("x=%f,y=%f,z=%f", msg.x,msg.y,msg.z);
	xyz.x=msg.x;
	xyz.y=msg.y;
	xyz.z=msg.z;
	if (xyz.x!=0)
	{
		//init a point
		visualization_msgs::Marker points;

		//frame ID and timestamp
		points.header.frame_id= "/my_frame";
		points.header.stamp = ros::Time::now();
		
		//namespace and id
		points.ns = "OpenCV_Marker_Point";
		points.id = 0;

		//marker type is point
		points.type = visualization_msgs::Marker::POINTS;

		// Set the marker action.  Options are ADD and DELETE
		points.action = visualization_msgs::Marker::ADD;

		// POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.2;
		points.scale.y = 0.2;
		points.scale.z = 0.2;

		// Points are green
		points.color.r = 0.0f;
		points.color.g = 1.0f;
		points.color.b = 0.0f;
		points.color.a = 1.0;

		//
		points.pose.position.x = 0;
		points.pose.position.y = 0;
		points.pose.position.z = 0;
		points.pose.orientation.x = 0.0;
		points.pose.orientation.y = 0.0;
		points.pose.orientation.z = 0.0;
		points.pose.orientation.w = 1.0;	
		
		//
		
		geometry_msgs::Point p;
		p.x = -xyz.x;
		p.y = -xyz.y;
		p.z = -xyz.z;
		
		//
		points.points.push_back(p);
		trackerpoints.points.push_back(p);

		//
		marker_pub.publish(points);
		marker_pub.publish(trackerpoints);

	}

}


int main(int argc, char **argv)
{
	//name
	ros::init(argc, argv, "Rvizlistener");
	ros::NodeHandle n;

	//frame ID and timestamp
	trackerpoints.header.frame_id= "/my_frame";
	trackerpoints.header.stamp = ros::Time::now();
	//namespace and id
	trackerpoints.ns = "OpenCV_Marker_Trackerpoints";
	trackerpoints.id = 1;
	//marker type is point
	trackerpoints.type = visualization_msgs::Marker::POINTS;
	// POINTS markers use x and y scale for width/height respectively
	trackerpoints.scale.x = 0.1;
	trackerpoints.scale.y = 0.1;
	trackerpoints.scale.z = 0.1;

	// Points are green
	trackerpoints.color.r = 0.0f;
	trackerpoints.color.g = 0.0f;
	trackerpoints.color.b = 1.0f;
	trackerpoints.color.a = 1.0;

	// Set the marker action.  Options are ADD and DELETE
	trackerpoints.action = visualization_msgs::Marker::ADD;


	ros::Subscriber sub = n.subscribe("xyz", 1000, xyzCallback);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	ros::spin();
  return 0;
}
