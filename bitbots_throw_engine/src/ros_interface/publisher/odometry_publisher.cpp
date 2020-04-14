#include "ros_interface/publisher/odometry_publisher.h"
#include <nav_msgs/Odometry.h>

OdometryPublisher::OdometryPublisher(ros::NodeHandle & ros_node_handle)
{
    ros_publisher_odometry_ = ros_node_handle.advertise<nav_msgs::Odometry>("throw_odometry", 1);
    reset_counter();
}

void OdometryPublisher::reset_counter()
{
    counter_ = 1;
}

void OdometryPublisher::publish(uint16_t publish_factor_)
{
    if (counter_ > publish_factor_)
    {
        //TODO: implement
        // up_publisher_facade_->publish_odometry();

        reset_counter();
    }
    else
    {
        ++counter_;
    }
}