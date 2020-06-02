#ifndef BITBOTS_THROW_ROS_PUBLISHER_FACADE_H
#define BITBOTS_THROW_ROS_PUBLISHER_FACADE_H

#include "ros/ros.h"
#include "ros_interface/publisher/controler_command_publisher.h"
#include "ros_interface/publisher/debug_publisher.h"
#include "ros_interface/publisher/odometry_publisher.h"
#include "ros_interface/publisher/support_publisher.h"
#include "parameter/throw_node_parameter.h"

namespace bitbots_throw{
    class RosPublisherFacade{
    public:
        RosPublisherFacade(ros::NodeHandle & ros_node_handle, std::shared_ptr<ThrowNodeParameter> parameter);
        void prepare_publisher_for_throw();

        void publish_throw(bitbots_splines::JointGoals & joint_goals);
        void publish_odometry();
        void publish_debug(ThrowResponse response, bitbots_splines::JointGoals joint_goals);

    private:
        std::shared_ptr<DebugPublisher> sp_debug_publisher_;
        std::shared_ptr<SupportPublisher> sp_support_publisher_;
        std::shared_ptr<OdometryPublisher> sp_odometry_publisher_;
        std::shared_ptr<ControllerCommandPublisher> sp_controller_command_publisher_;

        std::shared_ptr<ThrowNodeParameter> sp_node_parameter_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_ROS_PUBLISHER_FACADE_H