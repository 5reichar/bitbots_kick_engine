#include "ros_interface/publisher/support_publisher.h"
#include <std_msgs/Char.h>

SupportPublisher::SupportPublisher(ros::NodeHandle & ros_node_handle)
{
    ros_publisher_support_ = ros_node_handle.advertise<std_msgs::Char>("throw_support_state", 1);
}

void SupportPublisher::publish()
{
    //TODO: implement
}