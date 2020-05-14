#ifndef SUPPORT_PUBLISHER_H
#define SUPPORT_PUBLISHER_H

#include "ros/ros.h"

class SupportPublisher
{
    //TODO: cleanup
public:
    SupportPublisher(ros::NodeHandle & ros_node_handle);

    virtual void publish();
    virtual void publish(char const support_foot);

private:
    ros::Publisher ros_publisher_support_;
    char current_support_foot_;
};

#endif