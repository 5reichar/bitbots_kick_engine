#ifndef BITBOTS_THROW_ODOMETRY_PUBLISHER_H
#define BITBOTS_THROW_ODOMETRY_PUBLISHER_H

#include "ros/ros.h"

namespace bitbots_throw{
    class OdometryPublisher{
    public:
        OdometryPublisher(ros::NodeHandle & ros_node_handle);

        void publish(uint16_t publish_factor_);
        void reset_counter();

    private:
        ros::Publisher ros_publisher_odometry_;
        uint16_t counter_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_ODOMETRY_PUBLISHER_H