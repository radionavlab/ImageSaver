// Author: Tucker Haydon

#include "buffer.h"

OdometryBuffer odometry_buffer = OdometryBuffer();
std::mutex OdometryBuffer::mtx_;
