#ifndef KICKENGINENODE_HPP
#define KICKENGINENODE_HPP

#include "KickEngine.hpp"
#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>

class KickEngineNode
{
public:
    KickEngineNode(/* args */);
    ~KickEngineNode();

    void kick();

private:
    void initialise_ros_subcribtions();
    void initialise_ros_publisher();
    void kick_ball(geometry_msgs::Vector3 const &const ball_position, geometry_msgs::Vector3 const &const target_position);
    void publish_kick() const;
    void publish_odemetry() const;
    void publish_controler_commands(std::vector<std::string> joint_names, std::vector<double> positions) const;

    void robot_state_callback(const humanoid_league_msgs::RobotControlState msg);
    void kick_callback();

    KickEngine m_kick_enginge;
    uint16_t m_uint_odometry_publish_factor;

    ros::NodeHandle m_ros_node_handle;

    ros::Publisher m_ros_publisher_controller_command;
    ros::Publisher m_ros_publisher_odometry;
    ros::Publisher m_ros_publisher_support;
    ros::Publisher m_ros_publisher_debug;
    ros::Publisher m_ros_publisher_debug_marker;

    ros::Subscriber m_ros_subsciber_kick;
    ros::Subscriber m_ros_subsciber_robot_state;

    robot_model::RobotModelPtr m_kinematic_model;
    robot_state::RobotStatePtr m_goal_state;
    robot_state::RobotStatePtr m_current_state;

    bitbots_ik::BioIKSolver m_bio_ik_solver;
};

#endif