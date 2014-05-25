////
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <irobot_tracker/trackInfo.h>
#include "beginner_tutorials/xy.h"

#define SCALE 10
visualization_msgs::Marker trackerPoints;
ros::Publisher position_pub;

//ros::Publisher xy_pub;
beginner_tutorials::xy myxy2;

float f;

/*
void xyCallback(const beginner_tutorials::xy& myxy)
{
  ROS_INFO("recieve:%d,%d", myxy.x,myxy.y);
	beginner_tutorials::xy myxy2;
	myxy2.x=myxy.x+100;
	myxy2.y=myxy.y+100;
	xy_pub.publish(myxy2);

  ROS_INFO("send:%d,%d", myxy2.x,myxy2.y);
}
*/
void callposition(const irobot_tracker::trackInfo& position);

int main(int argc, char** argv) {
	f=0.0;
	ros::init(argc, argv, "RvizL2Ttest");

	ros::NodeHandle nh;

	position_pub = nh.advertise<visualization_msgs::Marker>("trackPosition", 100);

	//xy_pub= nh.advertise<beginner_tutorials::xy>("Myxy2inRvizcall", 1000);


	ros::Subscriber sub = nh.subscribe("board_pose", 1000, callposition);
	ros::spin();
 
     	return 0;
}


void callposition(const irobot_tracker::trackInfo& position)
{



        //outMsg.pose.orientation.w = Q[0];
        //outMsg.pose.orientation.x = Q[1];
        //outMsg.pose.orientation.y = Q[2];
        //outMsg.pose.orientation.z = Q[3];
        //outMsg.pose.position.x = posX/100.0f;
        //outMsg.pose.position.y = posY/100.0f;
        //outMsg.pose.position.z = posZ/100.0f;
	float x=position.pose.position.x*SCALE;
	float y=position.pose.position.y*SCALE;
	float z=position.pose.position.z*SCALE;

	ROS_INFO("Get_trackInfo: x=%f,y=%f,z=%f",x,y,z);

	//init a point
	visualization_msgs::Marker positionPoints;

	//frame ID and timestamp
	positionPoints.header.frame_id= "/plain_position";
	positionPoints.header.stamp = ros::Time::now();
	
	//namespace and id
	positionPoints.ns = "position_of_plain";
	positionPoints.id = 0;

	//marker type is point
	positionPoints.type = visualization_msgs::Marker::POINTS;

	// Set the marker action.  Options are ADD and DELETE
	positionPoints.action = visualization_msgs::Marker::ADD;

	// POINTS markers use x and y scale for width/height respectively
	positionPoints.scale.x = 0.2;
	positionPoints.scale.y = 0.2;
	positionPoints.scale.z = 0.2;

	// Points are green
	positionPoints.color.r = 0.0f;
	positionPoints.color.g = 1.0f;
	positionPoints.color.b = 0.0f;
	positionPoints.color.a = 1.0;

	//

	positionPoints.pose.orientation.x = 0.0;
	positionPoints.pose.orientation.y = 0.0;
	positionPoints.pose.orientation.z = 0.0;
	positionPoints.pose.orientation.w = 1.0;


	////////////////////////////////////////////////
	//frame ID and timestamp
	trackerPoints.header.frame_id= "/plain_position";
	trackerPoints.header.stamp = ros::Time::now();
	//namespace and id
	trackerPoints.ns = "track_of_plain";
	trackerPoints.id = 1;
	//marker type is point
	trackerPoints.type = visualization_msgs::Marker::POINTS;
	// POINTS markers use x and y scale for width/height respectively
	trackerPoints.scale.x = 0.1;
	trackerPoints.scale.y = 0.1;
	trackerPoints.scale.z = 0.1;

	// Points are green
	trackerPoints.color.r = 0.0f;
	trackerPoints.color.g = 0.0f;
	trackerPoints.color.b = 1.0f;
	trackerPoints.color.a = 1.0;

	trackerPoints.pose.orientation.x = 0.0;
	trackerPoints.pose.orientation.y = 0.0;
	trackerPoints.pose.orientation.z = 0.0;
	trackerPoints.pose.orientation.w = 1.0;

	// Set the marker action.  Options are ADD and DELETE
	trackerPoints.action = visualization_msgs::Marker::ADD;
	
	
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	p.z = z;
	
	//
	positionPoints.points.push_back(p);
	trackerPoints.points.push_back(p);

	//
	position_pub.publish(positionPoints);
	position_pub.publish(trackerPoints);
/*
	
		//init a point
		visualization_msgs::Marker points;

		//frame ID and timestamp
		points.header.frame_id= "/plain_position";
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
		position_pub.publish(points);

		f++;
		if(f>30)f=0;
*/
/*
	myxy2.x=x+100;
	myxy2.y=y+100;
	xy_pub.publish(myxy2);
	ROS_INFO("send:%d,%d", myxy2.x,myxy2.y);
*/
}



