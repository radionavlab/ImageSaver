//Author: Tucker Haydon
#ifndef IMAGE_SAVER_BUFFER_H
#define IMAGE_SAVER_BUFFER_H

#include <mutex>
#include <nav_msgs/Odometry.h>
#include <iostream>

class OdometryBuffer {
  private:
    nav_msgs::Odometry odometry_;
    static std::mutex mtx_;

  public:
    OdometryBuffer() {};

    nav_msgs::Odometry GetOdometryMsg() {
      std::lock_guard<std::mutex> guard(OdometryBuffer::mtx_);
      std::cout << this->odometry_.pose.pose.position.x << std::endl;
      return odometry_; 
    };

    void SetOdometryMsg(const nav_msgs::Odometry& odometry) {
      std::lock_guard<std::mutex> guard(OdometryBuffer::mtx_);
      this->odometry_ = odometry;
    };
};

extern OdometryBuffer odometry_buffer;

#endif
