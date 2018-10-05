// Author: Tucker Haydon
#ifndef IMAGE_SAVER_ROS_HANDLER_H
#define IMAGE_SAVER_ROS_HANDLER_H

#include <nav_msgs/Odometry.h>

namespace image_saver {
  void OdometryMessageHandler(const nav_msgs::Odometry odometry_msg);
};

#endif
