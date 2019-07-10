#ifndef KICKENGINENODE_HPP
#define KICKENGINENODE_HPP

#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "KickEngineNodeService.hpp"
#include <humanoid_league_msgs/RobotControlState.h>

class KickEngineNode
{
	// TODO cleanup
public:
    KickEngineNode(/* args */);

private:
    void initialise_ros_subcribtions();
    void initialise_ros_publisher();

    void kick(geometry_msgs::Vector3 & ball_position, geometry_msgs::Vector3 & target_position);
    void kick_ball(geometry_msgs::Vector3 & ball_position, geometry_msgs::Vector3 & target_position);

    void publish_kick();
    void publish_odemetry();
    void publish_debug();
	void publish_marker(std::string name_space, std::string frame, geometry_msgs::Pose pose, std_msgs::ColorRGBA color);
    void publish_markers();
    virtual void publish_controler_commands(std::vector<std::string> joint_names, std::vector<double> positions);
    virtual void publish_controler_commands(std::vector<std::string> joint_names, std::vector<double> positions, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> max_currents);

    void robot_state_callback(const humanoid_league_msgs::RobotControlState msg);
    void kick_callback(const humanoid_league_msgs::Kick action);
	void reconfigure_callback(bitbots_kick_engine::bitbots_quintic_walk_paramsConfig& config, uint32_t level);

	geometry_msgs::Vector3 get_step_scale();
	geometry_msgs::Vector3 get_step_center_scale();
	geometry_msgs::Vector3 get_default_scale();
	geometry_msgs::Vector3 get_scale(float x, float y, float z);

	std_msgs::ColorRGBA get_color_left_feet();
	std_msgs::ColorRGBA get_color_right_feet();
	std_msgs::ColorRGBA get_color_fly_feet();
	std_msgs::ColorRGBA get_color_support_feet();
	std_msgs::ColorRGBA get_color_last_step();
	std_msgs::ColorRGBA get_color_next_step();
	std_msgs::ColorRGBA get_color(float red, float green, float blue, float alpha);

	void set_debug(bool debug);

	bool m_bool_debug;
	int32_t m_int_marker_id;
	uint16_t m_uint_odometry_publish_factor;

	KickEngineNodeService m_node_service;

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