/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	irobot ̽��
* Brief: ̽��4�Σ��ֱ�canny��threshold������ƥ�䡣
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/4/6 21:00
* History:
************************************************************************/
// opencv lib
#include <opencv2/opencv.hpp>
// ros 
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//msg
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

#include "ukftest/markerInfo.h"
// my class
#include "IrobotDetector.h"

using namespace cv;
using namespace std;

cv_bridge::CvImagePtr cv_ptr;
Mat InImage;

ukftest::markerInfo outMsg;

ros::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg);

/////////////////////////////////////////////////////////
//mine
IrobotDetector detect1;
////////////////////////////////////////////////////////

int main(int argc,char **argv)
{
	ros::init(argc, argv, "neg_T_test_node");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber imgSub;

	imgSub = it.subscribe("/uav_cam/image", 1, imageCallback);
	pub = nh.advertise<ukftest::markerInfo>("marker_pose",200);
	ros::spin();
	return 0;
}





void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	Mat frame;
	cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
	frame = cv_ptr->image;
	resize(frame,frame,Size(320,240));
	cvtColor(frame, frame, CV_GRAY2RGB);
	/////////////////////////////////////////////////////////////////////////////////
	//mine 
	detect1.processFrame(frame);

	imshow("debug",frame);
	waitKey(1);
	/////////////////////////////////////////////////////////////////////
	// get board position
	if (detect1.isTracking == false)
	{
		outMsg.header.stamp =ros::Time::now();
		outMsg.header.frame_id = "camera";
		outMsg.isTracking = 0;
		pub.publish(outMsg);
		return;
	}
	else
	{
		outMsg.header.stamp =ros::Time::now();
		outMsg.header.frame_id = "camera";
		outMsg.isTracking = 1;


		float x = detect1.pCenter.x;
		float y = detect1.pCenter.y;

	
		line(frame,Point2f(160,120),Point2f(x,y),Scalar(255,255,0),1);
		circle(frame,Point2f(x,y),3,Scalar(0,255,255),4,8);
		imshow("debug",frame);
		waitKey(1);
		// undistort
		Mat K=(Mat_<float>(3,3)<<
			180.2346315,0,159.5,
			0,180.2346315,119.5,
			0,0,1);

		Mat distCoeff = (Mat_<float>(5,1) << -0.276089956,0.1087447314,0.0,0.0,-0.024036442);

		Mat src(1,1,CV_32FC2);
		Mat dst(1,1,CV_32FC2);
		src.ptr<float>(0)[0] = x;
		src.ptr<float>(0)[1] = y;
	
		undistortPoints(src,dst,K,distCoeff);
	
		x = dst.ptr<float>(0)[0] * K.at<float>(0,0) + K.at<float>(0,2);
		y = dst.ptr<float>(0)[1] * K.at<float>(1,1) + K.at<float>(1,2);

		outMsg.uv1.x = x;
		outMsg.uv1.y = y;
		outMsg.uv1.z = 1.0f;

		pub.publish(outMsg);

		
	}
}
