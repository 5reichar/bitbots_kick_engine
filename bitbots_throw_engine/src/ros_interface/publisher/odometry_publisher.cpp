#include "ros_interface/publisher/odometry_publisher.h"
#include <nav_msgs/Odometry.h>

namespace bitbots_throw{
    OdometryPublisher::OdometryPublisher(ros::NodeHandle & ros_node_handle){
        ros_publisher_odometry_ = ros_node_handle.advertise<nav_msgs::Odometry>("throw_odometry", 1);
        reset_counter();
    }

    void OdometryPublisher::reset_counter(){
        counter_ = 1;
    }

    void OdometryPublisher::publish(uint16_t publish_factor_){
        if (counter_ > publish_factor_){
            //TODO: implement
            reset_counter();
        }
        else{
            ++counter_;
        }
    }
} //bitbots_throw