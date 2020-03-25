#ifndef ROS_PUBLISHER_FACADE_H
#define ROS_PUBLISHER_FACADE_H

#include "ros/ros.h"
#include "ros_interface/publisher/controler_command_publisher.h"
#include "ros_interface/publisher/debug_publisher.h"
#include "ros_interface/publisher/odometry_publisher.h"
#include "ros_interface/publisher/support_publisher.h"

class RosPublisherFacade
{
    // TODO cleanup
public:
    RosPublisherFacade(ros::NodeHandle & ros_node_handle);

    void publish_throw(bool debug_active = false);
    void publish_odometry();

private:
    std::shared_ptr<DebugPublisher> sp_debug_publisher_;
    std::shared_ptr<SupportPublisher> sp_support_publisher_;
    std::shared_ptr<OdometryPublisher> sp_odometry_publisher_;
    std::shared_ptr<ControllerCommandPublisher> sp_controller_command_publisher_;
};

#endif