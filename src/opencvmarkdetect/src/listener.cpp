#include "ros/ros.h"
#include "opencvmarkdetect/Xyz.h"

void xyzCallback(const opencvmarkdetect::Xyz& msg)
{
  ROS_INFO("x=%f,y=%f,z=%f", msg.x,msg.y,msg.z);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("xyz", 1000, xyzCallback);
  ros::spin();

  return 0;
}
