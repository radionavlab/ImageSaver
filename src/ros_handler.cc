// Author: Tucker Haydon
#include "ros_handler.h"
#include "buffer.h"

namespace image_saver {

  // Extern Variable
  OdometryBuffer odometry_buffer;
  
  void OdometryMessageHandler(const nav_msgs::Odometry odometry_msg) {
    odometry_buffer.SetOdometryMessage(odometry_msg);
  }

};
