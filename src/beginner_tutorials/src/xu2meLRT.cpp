#include "ros/ros.h"
#include "beginner_tutorials/xy.h"

void xyCallback(const beginner_tutorials::xy& myxy)
{
  ROS_INFO("%d,%d", myxy.x,myxy.y);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "xu2meLRT");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("Myxy2", 1000, xyCallback);
  ros::spin();
  return 0;
}

