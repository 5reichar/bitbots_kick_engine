#ifndef SUPPORT_PUBLISHER_H
#define SUPPORT_PUBLISHER_H

#include "ros/ros.h"

class SupportPublisher
{
    // TODO cleanup
public:
    SupportPublisher(ros::NodeHandle & ros_node_handle);

    void publish();

private:
    ros::Publisher ros_publisher_support_;
};

#endif