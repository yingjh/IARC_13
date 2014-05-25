/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	irobot Ì½²â
* Brief: Ì½²â4´Î£¬·Ö±ðcanny£¬threshold¼ÓÂÖÀªÆ¥Åä¡£
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/4/6 21:00
* History:
************************************************************************/
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "irobot_tracker/trackInfo.h"
#include "irobot_tracker/uav_math.h"
#include "geometry_msgs/Vector3.h"
#include <opencv2/opencv.hpp>
// my class
#include "IrobotDetector.h"

using namespace cv;
using namespace std;

#define THRESHOLD 80
#define SCALE 2
/* ****CONST DEFINITION**** */
//
//      * ------------------->  x
//      |
//      |
//      |
//     \/ y 
// below are the positions of markers relative to home point

cv_bridge::CvImagePtr cv_ptr;
Mat InImage;

float posX, posY, posZ;
Mat pos_tmp;
float rvecX, rvecY, rvecZ;
float tmpf1, tmpf2, tmpf3;
float ortX, ortY, ortZ;
Mat R_ct;
float Q[4];
irobot_tracker::trackInfo outMsg;

ros::Publisher pub;

bool is_debug_on;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

/////////////////////////////////////////////////////////
//mine
IrobotDetector detect1;
////////////////////////////////////////////////////////

int main(int argc,char **argv)
{
	//////////////////////////////////////////////////////////////
	//mine

	////////////////////////////////////////
	ros::init(argc, argv, "webcam_identifier");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imgSub;

	nh.param("is_debug_on", is_debug_on, false);
	imgSub = it.subscribe("/uav_cam/image", 1, imageCallback);
	pub = nh.advertise<irobot_tracker::trackInfo>("board_pose",200);
	ros::spin();
	return 0;
}





void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	Mat frame;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	frame = cv_ptr->image;
	resize(frame,frame,Size(320,240));
	cvtColor(frame, frame, CV_GRAY2RGB);
	/////////////////////////////////////////////////////////////////////////////////
	//mine 
	detect1.processFrame(frame);
	/////////////////////////////////////////////////////////////////////
	// get board position
	if (detect1.isTracking == false)
	{
		outMsg.header.stamp =ros::Time::now();
		outMsg.header.frame_id = "world";
		outMsg.isTracking = 0;
		pub.publish(outMsg);
		return;
	}
	else
	{
		posX = posY = posZ = 0;
		ortX = ortY = ortZ = 0;
		rvecX = rvecY = rvecY = 0;


		Rodrigues(detect1.Rvec, R_ct);
		
		Mat R_tt1;
		R_tt1=(Mat_<float>(3,3)<<-1,0,0,
				       0,1,0,
				       0,0,-1);
		Mat R_ct1;
		R_ct1=R_ct*R_tt1;
		/*
		cout<<"R_ct1\n"<<R_ct1.at<float>(0,0)<<","
				<<R_ct1.at<float>(0,1)<<","
				<<R_ct1.at<float>(0,2)<<"\n"
				<<R_ct1.at<float>(1,0)<<","
				<<R_ct1.at<float>(1,1)<<","
				<<R_ct1.at<float>(1,2)<<"\n"
				<<R_ct1.at<float>(2,0)<<","
				<<R_ct1.at<float>(2,1)<<","
				<<R_ct1.at<float>(2,2)<<"\n";
		*/
		pos_tmp = -R_ct1.t()*Mat(detect1.Tvec);

		posX = pos_tmp.at<float>(0,0);
		posY = pos_tmp.at<float>(1,0);
		posZ = pos_tmp.at<float>(2,0);

		Mat rvec;
		Rodrigues(R_ct1,rvec);
		rvecX = rvec.at<float>(0,0);
		rvecY = rvec.at<float>(1,0); 
		rvecZ = rvec.at<float>(2,0);
		float theta = sqrt(rvecX*rvecX+rvecY*rvecY+rvecZ*rvecZ);
		Q[0] = cos(theta/2);
		Q[1] = rvecX/theta*sin(theta/2);
		Q[2] = rvecY/theta*sin(theta/2);
		Q[3] = rvecZ/theta*sin(theta/2);

		outMsg.pose.orientation.w = Q[0];
		outMsg.pose.orientation.x = Q[1];
		outMsg.pose.orientation.y = Q[2];
		outMsg.pose.orientation.z = Q[3];
		outMsg.pose.position.x = posX/1000.0f;  
		outMsg.pose.position.y = posY/1000.0f;
		outMsg.pose.position.z = posZ/1000.0f;
		outMsg.header.stamp =ros::Time::now();
		outMsg.header.frame_id = "world";
		outMsg.isTracking = 1; 
		pub.publish(outMsg);

		cout<<"T_t1c: "<<outMsg.pose.position.x<<","<<outMsg.pose.position.y<<","<<outMsg.pose.position.z<<endl;
		cout<<"T_ct: "<<detect1.Tvec[0]/10<<","<<detect1.Tvec[1]/10<<","<<detect1.Tvec[2]/10<<endl;
		
		cout<<"rvecX_o: "<<detect1.Rvec.at<float>(0,0)<<"rvecY "<<detect1.Rvec.at<float>(1,0)<<"rvecz "<<detect1.Rvec.at<float>(2,0)<<endl;
		cout<<"rvecX_n "<<rvecX<<"rvecY "<<rvecY<<"rvecz "<<rvecZ<<endl;
		cout<<"Q: "<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<endl;

	}
}
