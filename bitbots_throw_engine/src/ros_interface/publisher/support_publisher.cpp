#include "ros_interface/publisher/support_publisher.h"
#include <std_msgs/Char.h>

SupportPublisher::SupportPublisher(ros::NodeHandle & ros_node_handle)
{
    current_support_foot_ = '\0';
    ros_publisher_support_ = ros_node_handle.advertise<std_msgs::Char>("throw_support_foot_state", 1);
}

void SupportPublisher::publish()
{
    publish('d');
}

void SupportPublisher::publish(char const support_foot)
{
    //TODO: implement

    // publish if foot changed
    if(support_foot != current_support_foot_)
    {
        std_msgs::Char support_state;

        support_state.data = support_foot;
        current_support_foot_ = support_foot;

        ros_publisher_support_.publish(support_state);
    }
}