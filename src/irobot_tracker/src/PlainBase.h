/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	SampleDetector.h
* Brief: Rviz plugin,PlainBase class 
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/3/18 15:00
* History:
************************************************************************/

#pragma once
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>

#define PLAIN_SCALE 0.001
#define TRACK_SCALE 0.01
#define SPEED_SCALE 0.01
#define ACCEL_SCALE 0.01
#define MOD_SRC "package://irobot_tracker/meshes/myplain.stl"
#define FRAME_ID "/plain_state"

class PlainBase
{
public:
	PlainBase(geometry_msgs::Pose estp,geometry_msgs::Pose mesp,
		geometry_msgs::Vector3 speedv,geometry_msgs::Vector3 acceleratev,ros::Publisher puber);
	PlainBase();
	~PlainBase();

	

	void update(geometry_msgs::Pose est,geometry_msgs::Pose mes,
		geometry_msgs::Vector3 speed,geometry_msgs::Vector3 accelerate);

	void update(geometry_msgs::Pose est,geometry_msgs::Pose mes);

	void setPub(ros::Publisher puber);

	
		
private:
	geometry_msgs::Pose est;
	geometry_msgs::Pose mes;
	geometry_msgs::Vector3 speed;
	geometry_msgs::Vector3 accelerate;
	
	visualization_msgs::Marker plainMod;
	visualization_msgs::Marker estTrack;
	visualization_msgs::Marker mesTrack;
	visualization_msgs::Marker visualSpeed;
	visualization_msgs::Marker visualSpeedx;
	visualization_msgs::Marker visualSpeedy;
	visualization_msgs::Marker visualSpeedz;
	visualization_msgs::Marker visualAccelerate;
	
	visualization_msgs::Marker initPlainMod();
	visualization_msgs::Marker initTrack();
	visualization_msgs::Marker initVisualSpeed();
	visualization_msgs::Marker initVisualAccelerate();
	void updatePlainMod(geometry_msgs::Pose newPose, std::string ns, float r, float g, float b);

	void updateTrack(visualization_msgs::Marker & mytrack,geometry_msgs::Pose newPose, std::string ns, float r, float g, float b);

	void updateSpeed(geometry_msgs::Vector3 newSpeed,geometry_msgs::Pose newPose, std::string ns, float r, float g, float b);
	void updateAccelerate(geometry_msgs::Vector3 newAccelerate,geometry_msgs::Pose newPose, std::string ns, float r, float g, float b);
	ros::Publisher pub;

	void Quaternoin_to_Eular(const float Q[4], float *yaw, float *pitch, float *roll);
	void Eular_to_Quaternion(float Q[4], float yaw, float pitch, float roll);
	//void updateTrack(std::vector<geometry_msgs::Point>& points,geometry_msgs::Pose newPose, std::string ns, float r, float g, float b);
};
