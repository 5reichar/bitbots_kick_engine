#ifndef BITBOTS_THROW_SYSTEM_PUBLISHER_H
#define BITBOTS_THROW_SYSTEM_PUBLISHER_H

#include <string>
#include <ros/console.h>

namespace bitbots_throw{
    class SystemPublisher{
    public:
        static void publish_fatal(const std::string & message, std::string object_info = ""){
            ROS_FATAL_STREAM(create_string_for_ros(get_package_id(), object_info, message));
        };

        static void publish_error(const std::string & message, std::string object_info = ""){
            ROS_ERROR_STREAM(create_string_for_ros(get_package_id(), object_info, message));
        };

        static void publish_warning(const std::string & message, std::string object_info = ""){
            ROS_WARN_STREAM(create_string_for_ros(get_package_id(), object_info, message));
        };

        static void publish_info(const std::string & message, std::string object_info = ""){
            ROS_INFO_STREAM(create_string_for_ros(get_package_id(), object_info, message));
        };

        static void publish_debug(const std::string & message, std::string object_info = ""){
            ROS_DEBUG_STREAM(create_string_for_ros(get_package_id(), object_info, message));
        };

        static void publish_runtime_error(const std::runtime_error& error, std::string object_info = ""){
            ROS_ERROR_STREAM(create_string_for_ros(get_package_id(), object_info, error.what()));
        };

    private:
        static std::string get_package_id(){
            return "Bitbots Throw";
        };

        static std::string create_string_for_ros(std::string const & package_id, std::string const & object_info, std::string const & message){
            std::string ros_message = "[" + package_id;
            if (object_info != ""){
                ros_message += " : " + object_info;
            }
            ros_message += "]:  " + message;
            return ros_message;
        }
    };
} //bitbots_throw
#endif //BITBOTS_THROW_SYSTEM_PUBLISHER_H