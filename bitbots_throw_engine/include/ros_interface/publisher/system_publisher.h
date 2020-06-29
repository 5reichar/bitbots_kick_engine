#ifndef BITBOTS_THROW_SYSTEM_PUBLISHER_H
#define BITBOTS_THROW_SYSTEM_PUBLISHER_H

#include <string>
#include <ros/console.h>

namespace bitbots_throw{
    class SystemPublisher{
    public:
        static void publish_fatal(const std::string & message){
            ROS_FATAL_STREAM(message);
        };

        static void publish_error(const std::string & message){
            ROS_ERROR_STREAM(message);
        };

        static void publish_warning(const std::string & message){
            ROS_WARN_STREAM(message);
        };

        static void publish_info(const std::string & message){
            ROS_INFO_STREAM(message);
        };

        static void publish_debug(const std::string & message){
            ROS_DEBUG_STREAM(message);
        };

        static void publish_runtime_error(const std::runtime_error& error){
            ROS_ERROR_STREAM(error.what());
        };

    private:
    };
} //bitbots_throw
#endif //BITBOTS_THROW_SYSTEM_PUBLISHER_H