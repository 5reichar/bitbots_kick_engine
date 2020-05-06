#ifndef SYSTEM_PUBLISHER_H
#define SYSTEM_PUBLISHER_H

#include "ros/ros.h"
#include <string>

class SystemPublisher
{
    // TODO cleanup
public:
    //SystemPublisher(ros::NodeHandle & ros_node_handle);

    static void publish_warning(std::string messsage)
    {
        ROS_WARN(messsage);
    };

private:
    ros::Publisher ros_publisher_support_;
};

#endif