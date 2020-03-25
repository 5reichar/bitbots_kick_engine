#ifndef DUBUG_PUBLISHER_H
#define DUBUG_PUBLISHER_H

#include "ros/ros.h"

class DebugPublisher
{
    // TODO cleanup
public:
    DebugPublisher(ros::NodeHandle & ros_node_handle);

    void publish();
    void publish_markers();

private:
    ros::Publisher ros_publisher_debug_;
    ros::Publisher ros_publisher_debug_marker_;
};

#endif