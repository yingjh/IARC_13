#include <SerialStream.h>
#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include <unistd.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace LibSerial;

SerialStream serial_port;

int8_t yaw_rate = 1,pitch = 2,roll = 3;
int8_t vel = 4;

void serial_interrupt(int sig);
void initSerial();

int main (int argc, char** argv)
{
	signal(SIGINT, serial_interrupt);
	initSerial();
	ros::init(argc, argv, "serial_test");
	ros::NodeHandle nv;
	ros::Rate loop_rate(10);

	//capture
	cv_bridge::CvImagePtr cvPtr;
	cv_bridge::CvImage outMsg;
	cv::VideoCapture cap(-1);
	cv::Mat colorImg, grayImg;
	image_transport::ImageTransport it(nv);
	image_transport::Publisher pub = it.advertise("uav_cam/image", 3);

	while (ros::ok())
	{
		cap >> colorImg;
		cv::cvtColor(colorImg, grayImg, CV_RGB2GRAY);
		outMsg.image = grayImg;
		//cv::imshow("grayImg",grayImg);
		outMsg.header.stamp = ros::Time::now();
		outMsg.encoding = "mono8";
		pub.publish(outMsg.toImageMsg());
		int key;
		key=waitKey(100);
		printf("%d \n",key);
		//char key;
		//cin>>key;
		switch(key){
			case 'w':
				pitch++;
				break;
			case 's':
				pitch--;
				break;
			case 'q':
				yaw_rate++;
				break;
			case 'e':
				yaw_rate--;
				break;
			case 'a':
				roll++;
				break;
			case 'd':
				roll--;
				break;
			default:
				yaw_rate=0;
				roll=0;
				pitch=0;


		}
		printf("yaw %i pitch %i row %i\n", yaw_rate, pitch, roll);
		serial_port << "AA" << yaw_rate << pitch << roll << vel <<"BB";
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
		
}

void serial_interrupt(int sig){ // can be called asynchronously
	if (serial_port.IsOpen())
		serial_port.Close();
	ros::shutdown();
}

void initSerial()
{
	
	serial_port.Open( "/dev/ttySAC2" ) ;
	if ( ! serial_port.good() )
	{
	 std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
			   << "Error: Could not open serial port."
			   << std::endl ;
	 exit(1) ;
	}
	//
	// Set the baud rate of the serial port.
	//
	serial_port.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
	if ( ! serial_port.good() )
	{
	 std::cerr << "Error: Could not set the baud rate." <<  
	std::endl ;
	 exit(1) ;
	}
	//
	// Set the number of data bits.
	//
	serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
	if ( ! serial_port.good() )
	{
	 std::cerr << "Error: Could not set the character size." <<  
	std::endl ;
	 exit(1) ;
	}
	//
	// Disable parity.
	//
	serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
	if ( ! serial_port.good() )
	{
	 std::cerr << "Error: Could not disable the parity." <<  
	std::endl ;
	 exit(1) ;
	}
	//
	// Set the number of stop bits.
	//
	serial_port.SetNumOfStopBits( 1 ) ;
	if ( ! serial_port.good() )
	{
	 std::cerr << "Error: Could not set the number of stop bits."
			   << std::endl ;
	 exit(1) ;
	}
	//
	// Turn off hardware flow control.
	//
	serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
	if ( ! serial_port.good() )
	{
	 std::cerr << "Error: Could not use hardware flow control."
			   << std::endl ;
	 exit(1) ;
	}
}
