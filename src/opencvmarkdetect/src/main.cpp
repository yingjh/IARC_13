/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	main.cpp
* Brief: opencvÊ¶±ð 
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/2/15 22:00
* History:
************************************************************************/

#include "Marker.h"
#include "MarkerDetector.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <bitset>
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "opencvmarkdetect/Xyz.h"

using namespace cv;
using namespace std;

VideoCapture initCam()
{
	VideoCapture camCapture;
	if (!camCapture.open(0))
	{
		std::cout << "can't open cam!" << std::endl;
	}
	camCapture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	camCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	return camCapture;
}

int main(int argc, char **argv)
{
	/*
	//{01001}*5µÄ²âÊÔ
	Mat test=Mat::zeros(7, 7, CV_8UC1);
	test.at<uchar>(1,2)=1;
	test.at<uchar>(2,2)=1;
	test.at<uchar>(3,2)=1;
	test.at<uchar>(4,2)=1;
	test.at<uchar>(5,2)=1;
	test.at<uchar>(1,5)=1;
	test.at<uchar>(2,5)=1;
	test.at<uchar>(3,5)=1;
	test.at<uchar>(4,5)=1;
	test.at<uchar>(5,5)=1;

	Marker testM;
	int n;
	int code=testM.decode(test,n);
	printf("code:%d \n",code);
	std::cout<< "marker ID: " << std::bitset<10>(code) << std::endl<<"ratation index:"<<n<<std::endl;

	*/

	//ROS
	ros::init(argc, argv, "opencvMarkerdetect");
	ros::NodeHandle n;
	ros::Publisher xyz_pub = n.advertise<opencvmarkdetect::Xyz>("xyz", 1000);
	ros::Rate loop_rate(10);
	int count = 0;
	opencvmarkdetect::Xyz msg;


	//OpenCV
	Mat image;
	VideoCapture cam=initCam();
	MarkerDetector mark;

	while(ros::ok())
	{
		cam>>image;
		mark.processFrame(image);
		mark.drawMarkers(image);
		mark.drawCube(image);
		flip(image,image,1);
		imshow("AR",image);
		std::cout << "<<<<<<<<<<<<<<<---------new frame-------->>>>>>>>>>>>>>>" << std::endl;
		std::cout << "frame width:"<<image.cols<<"  "<<"frame height:"<<image.rows<<std::endl;
		if(mark.m_markers.size()>0)
		{
			std::cout<< "marker ID: " << std::bitset<10>(mark.m_markers[0].m_id) << std::endl;
			
			msg.x = mark.m_markers[0].m_translation[0];
			msg.y = mark.m_markers[0].m_translation[1];
			msg.z = mark.m_markers[0].m_translation[2];
		}
		else{
			msg.x = 0;
			msg.y = 0;
			msg.z = 0;
		}
		
		

		ROS_INFO("x=%f,y=%f,z=%f", msg.x,msg.y,msg.z);
		xyz_pub.publish(msg);
		
		ros::spinOnce();
		loop_rate.sleep();
		++count;


		
	}
	return 0;

}
