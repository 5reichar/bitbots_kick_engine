#include "ros_interface/publisher/debug_publisher.h"
#include <visualization_msgs/Marker.h>
#include <bitbots_throw_engine/throw_debug.h>
#include <bitbots_throw_engine/throw_engine_debug.h>

DebugPublisher::DebugPublisher(ros::NodeHandle & ros_node_handle)
{
    ros_publisher_debug_ = ros_node_handle.advertise<bitbots_throw_engine::throw_debug>("throw_debug", 1);
    ros_publisher_debug_engine_ = ros_node_handle.advertise<bitbots_throw_engine::throw_engine_debug>("throw_engine_debug", 1);
	ros_publisher_debug_marker_ = ros_node_handle.advertise<visualization_msgs::Marker>("throw_debug_marker", 1);
}

void DebugPublisher::publish_engine_debug(ThrowResponse response)
{
    //only do something if someone is listing
    if (ros_publisher_debug_engine_.getNumSubscribers() == 0 && ros_publisher_debug_marker_.getNumSubscribers() == 0)
    {
        return;
    }

    bitbots_throw_engine::throw_engine_debug msg;
    msg.header.stamp = ros::Time::now();

    // times
    msg.phase_time = response.phase_;
    msg.traj_time = response.traj_time_;

    ros_publisher_debug_engine_.publish(msg);
}

void DebugPublisher::publish_ik_debug(ThrowResponse response, bitbots_splines::JointGoals joint_goals)
{
    //only do something if someone is listing
    if (ros_publisher_debug_.getNumSubscribers() == 0 && ros_publisher_debug_marker_.getNumSubscribers() == 0)
    {
        return;
    }

    bitbots_throw_engine::throw_debug msg;

    tf2::toMsg(response.support_foot_to_left_hand_, msg.left_hand_goal);
    tf2::toMsg(response.support_foot_to_right_hand_, msg.right_hand_goal);
    tf2::toMsg(response.support_foot_to_trunk_, msg.trunk_goal);

    publish_arrow_marker("throw_engine_left_hand_goal", "base_link", msg.left_hand_goal, 0, 1, 0, 1);
    publish_arrow_marker("throw_engine_right_hand_goal", "base_link", msg.right_hand_goal, 1, 0, 0, 1);
    publish_arrow_marker("throw_engine_trunk_goal", "base_link", msg.trunk_goal, 0, 0, 1, 1);

    ros_publisher_debug_.publish(msg);
}

void DebugPublisher::publish_arrow_marker(std::string name_space, std::string frame, geometry_msgs::Pose pose, float r, float g, float b, float a)
{
    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = frame;

    marker_msg.type = marker_msg.ARROW;
    marker_msg.ns = name_space;
    marker_msg.action = marker_msg.ADD;
    marker_msg.pose = pose;

    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    marker_msg.color = color;

    geometry_msgs::Vector3 scale;
    scale.x = 0.01;
    scale.y = 0.003;
    scale.z = 0.003;
    marker_msg.scale = scale;

    marker_msg.id = marker_id_;
    marker_id_++;

    ros_publisher_debug_marker_.publish(marker_msg);
}

void DebugPublisher::publish_throw_markers(ThrowResponse response)
{
    //publish markers
    visualization_msgs::Marker marker_msg;
    marker_msg.header.stamp = ros::Time::now();

    ros_publisher_debug_marker_.publish(marker_msg);

    marker_id_++;
}
