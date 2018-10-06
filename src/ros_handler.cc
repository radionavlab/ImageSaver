// Author: Tucker Haydon
#include "ros_handler.h"
#include "buffer.h"

// Extern Variable
OdometryBuffer odometry_buffer;

namespace image_saver {
  
  void OdometryMessageHandler(const nav_msgs::Odometry odometry_msg) {
    odometry_buffer.SetOdometryMsg(odometry_msg);
  }

};
