#ifndef BITBOTS_THROW_DUBUG_PUBLISHER_H
#define BITBOTS_THROW_DUBUG_PUBLISHER_H

#include "ros/ros.h"
#include <moveit_msgs/RobotState.h>
#include "bitbots_splines/abstract_visualizer.h"
#include "bitbots_splines/abstract_ik.h"

#include "utility/throw_utilities.h"

namespace bitbots_throw{
    class DebugPublisher : public bitbots_splines::AbstractVisualizer{
    public:
        explicit DebugPublisher(ros::NodeHandle & ros_node_handle, std::string topic, std::string topic_engine, std::string topic_marker);

        void publish_engine_debug(ThrowResponse response);
        void publish_ik_debug(ThrowResponse response, bitbots_splines::JointGoals joint_goals);
        void publish_throw_markers(ThrowResponse response);

    private:
        void publish_arrow_marker(std::string name_space, std::string frame, geometry_msgs::Pose pose, float r, float g, float b, float a);

        int32_t marker_id_;

        ros::Publisher ros_publisher_debug_;
        ros::Publisher ros_publisher_debug_engine_;
        ros::Publisher ros_publisher_debug_marker_;

        robot_model::RobotModelPtr kinematic_model_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_DUBUG_PUBLISHER_H