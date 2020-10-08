#ifndef BITBOTS_THROW_ROS_PUBLISHER_FACADE_H
#define BITBOTS_THROW_ROS_PUBLISHER_FACADE_H

#include "ros/ros.h"
#include "ros_interface/throw_visualizer.h"
#include "ros_interface/publisher/controler_command_publisher.h"
#include "ros_interface/publisher/debug_publisher.h"
#include "ros_interface/publisher/odometry_publisher.h"
#include "ros_interface/publisher/support_publisher.h"
#include "parameter/throw_engine_parameter.h"
#include "engine/throw_engine.h"

namespace bitbots_throw{
    class RosPublisherFacade{
    public:
        struct RosPublisherTopics{
            std::string str_controller_command_topic_;
            std::string str_support_topic_;
            std::string str_odometry_topic_;
            std::string str_debug_topic_;
            std::string str_debug_marker_topic_;
            std::string str_debug_visualization_base_topic_;
        };

        RosPublisherFacade(ros::NodeHandle & ros_node_handle
                          ,std::shared_ptr<ThrowEngineParameter> & sp_engine_parameter
                          ,RosPublisherTopics const & topics
                          ,ThrowVisualizer::ThrowVisualizerParams const & visualization_parameter
                          ,std::shared_ptr<ThrowDebugParameter> & sp_debug_parameter);
        void prepare_publisher_for_throw();
        void set_parameter(std::shared_ptr<ThrowEngineParameter> & sp_engine_parameter, std::shared_ptr<ThrowDebugParameter> & sp_debug_parameter);

        void publish_throw(bitbots_splines::JointGoals & joint_goals);
        void publish_odometry();
        void publish_engine_debug(ThrowEngine * engine, ThrowRequest const & request, std::vector<ThrowResponse> & responses);
        void publish_debug(ThrowResponse const & response, int8_t const & percentage_done, int8_t const & movement_stage);
        void visualize_engine(ThrowEngine * engine);

    private:
        std::string build_data_from_transform(tf2::Transform & transform);

        std::shared_ptr<DebugPublisher> sp_debug_publisher_;
        std::shared_ptr<SupportPublisher> sp_support_publisher_;
        std::shared_ptr<OdometryPublisher> sp_odometry_publisher_;
        std::shared_ptr<ControllerCommandPublisher> sp_controller_command_publisher_;

        std::shared_ptr<ThrowVisualizer> sp_visualizer_;
        std::shared_ptr<ThrowDebugParameter> sp_debug_parameter_;
        std::shared_ptr<ThrowEngineParameter> sp_engine_parameter_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_ROS_PUBLISHER_FACADE_H