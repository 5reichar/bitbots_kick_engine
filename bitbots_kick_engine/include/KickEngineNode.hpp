#ifndef KICKENGINENODE_HPP
#define KICKENGINENODE_HPP

#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "KickEngineNodeService.hpp"
#include "KickEngineDebugService.hpp"
#include <humanoid_league_msgs/RobotControlState.h>
#include <bitbots_kick_engine/WalkingDebug.h>
#include <bitbots_kick_engine/KickAction.h>


class [[deprecated]] KickEngineNode
{
    // TODO cleanup
public:
    KickEngineNode(/* args */);
	~KickEngineNode();

private:
    void kickBall(geometry_msgs::Vector3 ball_position, geometry_msgs::Vector3 target_position);

	void kickCallback(const bitbots_kick_engine::KickAction action);
	void robotStateCallback(const humanoid_league_msgs::RobotControlState msg);
	void reconfigureKickPreparationPositions(bitbots_kick_engine::kick_preparation_positionsConfig &config, uint32_t level);
	void reconfigureKickEngineCallback(bitbots_kick_engine::kick_engine_paramsConfig& config, uint32_t level);
	void reconfigureKicksCallback(bitbots_kick_engine::kick_paramsConfig& config, uint32_t level);

    void initialiseRosPublisher();
    void initialiseRosSubcribtions();

    void publishKick();
    void publishOdemetry();
    void publishControlerCommands();

    void publishDebug();
    void publishMarkers();
    void publishMarker(std::string name_space, std::string frame, geometry_msgs::Pose pose, std_msgs::ColorRGBA color);

	bitbots_kick_engine::WalkingDebug createDebugMessage();

    int32_t m_int_marker_id;
    uint16_t m_uint_odometry_publish_factor;

    KickEngineNodeService * m_p_node_service;
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