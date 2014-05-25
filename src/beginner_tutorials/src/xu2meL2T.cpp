#include "ros/ros.h"
#include "beginner_tutorials/xy.h"


ros::Publisher xy_pub;
void xyCallback(const beginner_tutorials::xy& myxy)
{
  ROS_INFO("recieve:%d,%d", myxy.x,myxy.y);
	beginner_tutorials::xy myxy2;
	myxy2.x=myxy.x+100;
	myxy2.y=myxy.y+100;
	xy_pub.publish(myxy2);

  ROS_INFO("send:%d,%d", myxy2.x,myxy2.y);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "xu2meL2T");
  ros::NodeHandle n;
  xy_pub= n.advertise<beginner_tutorials::xy>("Myxy2", 1000);
  ros::Subscriber sub = n.subscribe("Myxy", 1000, xyCallback);
  ros::spin();
  return 0;
}
