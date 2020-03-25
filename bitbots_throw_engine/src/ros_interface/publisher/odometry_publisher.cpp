#include "ros_interface/publisher/odometry_publisher.h"
#include <nav_msgs/Odometry.h>

OdometryPublisher::OdometryPublisher(ros::NodeHandle & ros_node_handle)
{
    ros_publisher_odometry_ = ros_node_handle.advertise<nav_msgs::Odometry>("throw_odometry", 1);
}

void OdometryPublisher::publish()
{
    //TODO: implement
}