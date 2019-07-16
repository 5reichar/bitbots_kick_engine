#ifndef KICKENGINENODE_HPP
#define KICKENGINENODE_HPP

#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "KickEngineNodeService.hpp"
#include "KickEngineDebugService.hpp"
#include <humanoid_league_msgs/RobotControlState.h>

class KickEngineNode
{
    // TODO cleanup
public:
    KickEngineNode(/* args */);

private:
    void initialise_ros_subcribtions();
    void initialise_ros_publisher();

    void kick_callback(const humanoid_league_msgs::Kick action);
    void robot_state_callback(const humanoid_league_msgs::RobotControlState msg);
    void reconfigure_callback(bitbots_kick_engine::bitbots_quintic_walk_paramsConfig &config, uint32_t level);

    void kick_ball(geometry_msgs::Vector3 &ball_position, geometry_msgs::Vector3 &target_position);

    void publish_kick();
    void publish_odemetry();
    void publish_controler_commands();

    void publish_debug();
    void publish_markers();
    void publish_marker(std::string name_space, std::string frame, geometry_msgs::Pose pose, std_msgs::ColorRGBA color);

    geometry_msgs::Vector3 get_scale(float x, float y, float z);
    std_msgs::ColorRGBA get_color(float red, float green, float blue, float alpha);
    bitbots_quintic_walk::WalkingDebug create_debug_message();

    int32_t m_int_marker_id;
    uint16_t m_uint_odometry_publish_factor;

    KickEngineNodeService m_node_service;
    std::shared_ptr<KickEngineDebugService> m_sp_debug_service;

    ros::NodeHandle m_ros_node_handle;

    ros::Publisher m_ros_publisher_controller_command;
    ros::Publisher m_ros_publisher_odometry;
    ros::Publisher m_ros_publisher_support;
    ros::Publisher m_ros_publisher_debug;
    ros::Publisher m_ros_publisher_debug_marker;

    ros::Subscriber m_ros_subsciber_kick;
    ros::Subscriber m_ros_subsciber_robot_state;
};

#endif