#ifndef ODOMETRY_PUBLISHER_H
#define ODOMETRY_PUBLISHER_H

#include "ros/ros.h"

class OdometryPublisher
{
    // TODO cleanup
public:
    OdometryPublisher(ros::NodeHandle & ros_node_handle);

    void publish();

private:
    ros::Publisher ros_publisher_odometry_;
};

#endif