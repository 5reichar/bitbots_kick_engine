#ifndef BITBOTS_THROW_THROW_LOGGER_H
#define BITBOTS_THROW_THROW_LOGGER_H

#include <map>
#include <strstream>

#include "ros_interface/publisher/system_publisher.h"

namespace bitbots_throw{
    class ThrowLogger{
    public:
        ThrowLogger(bool const & active, std::string const additional_publish_info = "")
            : active_(active)
            , value_delimeter_("; ")
            , additional_publish_info_(additional_publish_info){
        };

        void log(std::string log_name, std::string value){
            logging_[log_name] << value_delimeter_ << value;
        };

        void publish_to_ros(std::string const log_name){
            SystemPublisher::publish_info(log_name + value_delimeter_ + logging_[log_name].str(), additional_publish_info_);
        };

        void publish_to_ros(){
            for(auto const & it : logging_){
                SystemPublisher::publish_info(it.first + value_delimeter_ + it.second.str(), additional_publish_info_);
            }
        };

    private:
        const bool active_;
        const std::string value_delimeter_;
        const std::string additional_publish_info_;
        std::map<std::string, std::stringstream> logging_;
    };
}

#endif //BITBOTS_THROW_THROW_LOGGER_H
