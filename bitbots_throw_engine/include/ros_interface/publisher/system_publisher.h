#ifndef SYSTEM_PUBLISHER_H
#define SYSTEM_PUBLISHER_H

#include <string>
#include <ros/console.h>

class SystemPublisher
{
    // TODO cleanup
public:
    //SystemPublisher(ros::NodeHandle & ros_node_handle);

    static void publish_warning(std::string messsage)
    {
        ROS_WARN_STREAM(messsage);
    };

private:
    //ros::Publisher ros_publisher_support_;
};

#endif