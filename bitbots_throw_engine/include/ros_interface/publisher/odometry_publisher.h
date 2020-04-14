#ifndef ODOMETRY_PUBLISHER_H
#define ODOMETRY_PUBLISHER_H

#include "ros/ros.h"

class OdometryPublisher
{
    // TODO cleanup
public:
    OdometryPublisher(ros::NodeHandle & ros_node_handle);

    void publish(uint16_t publish_factor_);
    void reset_counter();

private:
    ros::Publisher ros_publisher_odometry_;
    uint16_t counter_;
};

#endif