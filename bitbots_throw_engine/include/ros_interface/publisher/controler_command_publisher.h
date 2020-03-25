#ifndef CONTROLER_COMMAND_PUBLISHER_H
#define CONTROLER_COMMAND_PUBLISHER_H

#include "ros/ros.h"

class ControllerCommandPublisher
{
    // TODO cleanup
public:
    ControllerCommandPublisher(ros::NodeHandle & ros_node_handle);

    void publish();

private:
    ros::Publisher ros_publisher_controller_command_;
};

#endif