#ifndef BITBOTS_THROW_SUPPORT_PUBLISHER_H
#define BITBOTS_THROW_SUPPORT_PUBLISHER_H

#include "ros/ros.h"

namespace bitbots_throw{
    class SupportPublisher{
    public:
        SupportPublisher(ros::NodeHandle & ros_node_handle);

        virtual void publish();
        virtual void publish(char const support_foot);

    private:
        ros::Publisher ros_publisher_support_;
        char current_support_foot_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_SUPPORT_PUBLISHER_H