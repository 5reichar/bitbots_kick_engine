#ifndef SYSTEM_PUBLISHER_H
#define SYSTEM_PUBLISHER_H

#include <string>
#include <ros/console.h>

class SystemPublisher
{
    //TODO: cleanup
public:
    static void publish_fatal(const std::string & messsage)
    {
        ROS_FATAL_STREAM(messsage);
    };

    static void publish_error(const std::string & messsage)
    {
        ROS_ERROR_STREAM(messsage);
    };

    static void publish_warning(const std::string & messsage)
    {
        ROS_WARN_STREAM(messsage);
    };

    static void publish_info(const std::string & messsage)
    {
        ROS_INFO_STREAM(messsage);
    };

    static void publish_debug(const std::string & messsage)
    {
        ROS_DEBUG_STREAM(messsage);
    };

    static void publish_runtime_error(const std::runtime_error& error)
    {
        ROS_ERROR_STREAM(error.what());
    };


private:
};

#endif