#ifndef BITBOTS_THROW_THROW_NODE_H
#define BITBOTS_THROW_THROW_NODE_H

#include "ros/ros.h"
#include "utility/throw_struct.h"
#include "utility/throw_stabilizer_and_ik_factory.h"
#include "engine/throw_engine.h"
#include "parameter/throw_engine_parameter.h"

#include <dynamic_reconfigure/server.h>
#include <bitbots_throw/throw_action.h>
#include <tf2_ros/transform_listener.h>
#include <bitbots_throw/throw_paramsConfig.h>
#include "ros_interface/publisher/ros_publisher_facade.h"

namespace bitbots_throw{
    class ThrowNode
    {
    public:
        ThrowNode();

    private:
        // Initialization methods
        void set_default_parameter();
        void load_parameter();
        void init_ros_subscriptions();
        void init_dynamic_reconfiguration();
        void init_ik();

        // Callback methods
        void throw_callback(bitbots_throw::throw_action action);
        void throw_params_config_callback(throw_paramsConfig & config , uint32_t level);
        void throw_engine_params_config_callback(throw_engine_paramsConfig & config , uint32_t level);

        // Helper methods
        ThrowRequest create_throw_request(throw_action action);
        geometry_msgs::Pose get_pose(std::string const & frame_id
                                    ,double const & orientation = 1
                                    ,ros::Time const & time = ros::Time::now()
                                    ,std::string const & target_frame = "torso"
                                    ,ros::Duration const & timeout = ros::Duration(0.2));

        // member variables
        ThrowEngine throw_engine_;
        ThrowStabilizerAndIKFactory stabilizer_and_ik_;
        std::shared_ptr<RosPublisherFacade> sp_publisher_facade_;

        std::shared_ptr<ThrowDebugParameter> sp_debug_parameter_;
        std::shared_ptr<ThrowEngineParameter> sp_engine_parameter_;
        std::shared_ptr<RobotAndWorldParameter> sp_robot_and_world_parameter_;

        ros::NodeHandle ros_node_handle_;
        ros::Subscriber ros_subscriber_throw_;
        tf2_ros::Buffer tf2_ros_buffer_;
        tf2_ros::TransformListener tf2_ros_transform_listener_;
        dynamic_reconfigure::Server<throw_paramsConfig> dynamic_reconfigure_server_throw_params_;
        dynamic_reconfigure::Server<throw_engine_paramsConfig> dynamic_reconfigure_server_engine_params_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_NODE_H