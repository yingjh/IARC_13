/*
 * include ROS libraries
 */
#include "ros/ros.h"
#include "ros/console.h"

#include "irobot_tracker/posEstimator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_estimator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    irobot_tracker::posEstimator myEstimator(nh, nh_private);

    ROS_INFO("Start position filter\n");

    ros::spin();
}
