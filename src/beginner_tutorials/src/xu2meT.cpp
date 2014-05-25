#include "ros/ros.h"
#include "beginner_tutorials/xy.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "xu2meT");
  ros::NodeHandle n;
  ros::Publisher xy_pub = n.advertise<beginner_tutorials::xy>("Myxy", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {

    beginner_tutorials::xy myxy;
	myxy.x=count;
	myxy.y=count;
	

	//ROS_INFO("12312312");
    ROS_INFO("%d,%d", myxy.x,myxy.y);
    xy_pub.publish(myxy);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}

