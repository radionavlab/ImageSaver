#include "node.h"
#include "gps.h"

#include <string>

namespace image_saver {

Node::Node(int argc, char** argv) {
    ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName);
    this->nh_ = std::make_shared<ros::NodeHandle>("~");
    
    std::string pos_topic, att_topic;
    this->nh_->getParam("pos_topic", pos_topic);
    this->nh_->getParam("att_topic", att_topic);

    this->pos_sub_ = this->nh_->subscribe(pos_topic, 1, PositionMessageHandler);
    this->att_sub_ = this->nh_->subscribe(att_topic, 1, AttitudeMessageHandler);
}

void Node::Start() {
    ros::Rate r(20);
    while(this->nh_->ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
}; // namespace image_saver
