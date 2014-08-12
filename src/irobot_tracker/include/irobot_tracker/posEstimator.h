#ifndef POSESTIMATOR_H
#define POSESTIMATOR_H
/* 
 * include standard C++ libraries
 */
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

/*
 * include ROS libraries
 */
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/*
 * include ROS messages
 */
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "serial_to_uav/UAV.h"
#include "irobot_tracker/trackInfo.h"
#include "irobot_tracker/trackerDebug.h"

/*
 * include Eigen libraries, may be different on PC and XU
 */
#include <eigen3/Eigen/Dense>

/*
 * include libraries for communicate with GNUPLOT
 */
#include <boost/tuple/tuple.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#define GNUPLOT_ENABLE_PTY
#include "gnuplot-iostream.h"

/*
 * include Yu Yun math libraries
 */
#include "math_basic.h"
#include "math_vector.h"
#include "math_matrix.h"
#include "math_quaternion.h"
#include "math_rotation.h"

/*
 * include SRUKF 
 */ 
#include "irobot_tracker/SRUKF.h"

namespace irobot_tracker {
    class posEstimator
    {
        private:

        public:
    };
}

#endif
