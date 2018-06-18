// Author: Tucker Haydon
#pragma once

#include <memory>
#include <ros/ros.h>

namespace image_saver {

class Node {
public:
    Node(int argc, char** argv);
    void Start();

private:
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber att_sub_;
    ros::Subscriber pos_sub_;
};

}; // namespace image_saver
