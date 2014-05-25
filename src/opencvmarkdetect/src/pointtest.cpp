#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
	//name
	ros::init(argc, argv, "pointstest");
	ros::NodeHandle n;
	//name for RViz visualization_marker
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

	ros::Rate r(30);

	float f = 0.0;
	while (ros::ok())
	{

		//init a point
		visualization_msgs::Marker points;

		//frame ID and timestamp
		points.header.frame_id= "/my_frame";
		points.header.stamp = ros::Time::now();
		
		//namespace and id
		points.ns = "pointstest";
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
		p.x = f;
		p.y = f;
		p.z = f;
		
		//
		points.points.push_back(p);

		//
		marker_pub.publish(points);

		r.sleep();
		f++;
		if(f>30)f=0;
	}
}


		
			
