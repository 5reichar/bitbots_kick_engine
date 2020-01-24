#include "KickEngineNode.hpp"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <bitbots_msgs/JointCommand.h>

KickEngineNode::KickEngineNode()
{
	// TODO testing
	// TODO cleanup

	bool simulation = false;
	m_ros_node_handle.param<bool>("/simulation_active", simulation, false);

	m_p_node_service = new KickEngineNodeService(simulation);
	m_sp_debug_service = m_p_node_service->getDebugService();

	m_int_marker_id = 1;

	initialiseRosSubcribtions();
	initialiseRosPublisher();
}

KickEngineNode::~KickEngineNode()
{
	// TODO testing
	// TODO cleanup

	delete m_p_node_service;
}

void KickEngineNode::kickBall(geometry_msgs::Vector3 ball_position, geometry_msgs::Vector3 target_position)
{
	// TODO testing
	// TODO cleanup

	uint16_t odometry_counter = 1;
	ros::Rate loopRate(m_p_node_service->getEngineFrequence());

	while (ros::ok())
	{
		if (m_p_node_service->kick(ball_position, target_position))
		{
			publishKick();
		}

		if (odometry_counter > m_uint_odometry_publish_factor)
		{
			publishOdemetry();
			odometry_counter = 1;
		}
		else
		{
			++odometry_counter;
		}

		ros::spinOnce();
		loopRate.sleep();
	}
}

void KickEngineNode::robotStateCallback(const humanoid_league_msgs::RobotControlState msg)
{
	// TODO testing
	// TODO cleanup

	m_p_node_service->setRobotState(msg);
}

void KickEngineNode::kickCallback(const bitbots_kick_engine::KickAction action)
{
	// TODO testing
	// TODO cleanup

	kickBall(action.ball_pos, action.target);
}


void KickEngineNode::reconfigureKickPreparationPositions(bitbots_kick_engine::kick_preparation_positionsConfig &config, uint32_t level)
{
	// TODO testing
	// TODO cleanup

	m_p_node_service->reconfigureKickPreparationPositions(config, level);
}

void KickEngineNode::reconfigureKickEngineCallback(bitbots_kick_engine::kick_engine_paramsConfig& config, uint32_t level)
{
	// TODO testing
	// TODO cleanup

	m_sp_debug_service->setDebug(config.debugActive);
	m_uint_odometry_publish_factor = config.odomPubFactor;

	m_p_node_service->reconfigureEngineParameter(config, level);
}

void KickEngineNode::reconfigureKicksCallback(bitbots_kick_engine::kick_paramsConfig& config, uint32_t level)
{
	// TODO testing
	// TODO cleanup

	m_p_node_service->reconfigureKickParameter(config, level);
}

void KickEngineNode::initialiseRosPublisher()
{
	// TODO testing
	// TODO cleanup

	m_ros_publisher_controller_command = m_ros_node_handle.advertise<bitbots_msgs::JointCommand>("kick_motor_goals", 1);
	m_ros_publisher_odometry = m_ros_node_handle.advertise<nav_msgs::Odometry>("kick_odometry", 1);
	m_ros_publisher_support = m_ros_node_handle.advertise<std_msgs::Char>("kick_support_state", 1);

	m_ros_publisher_debug = m_ros_node_handle.advertise<bitbots_kick_engine::WalkingDebug>("kick_debug", 1);
	m_ros_publisher_debug_marker = m_ros_node_handle.advertise<visualization_msgs::Marker>("kick_debug_marker", 1);
}

void KickEngineNode::initialiseRosSubcribtions()
{
	// TODO testing
	// TODO cleanup

	m_ros_subsciber_kick = m_ros_node_handle.subscribe("kick", 1, &KickEngineNode::kickCallback, this, ros::TransportHints().tcpNoDelay());
	m_ros_subsciber_robot_state = m_ros_node_handle.subscribe("robot_state", 1, &KickEngineNode::robotStateCallback, this, ros::TransportHints().tcpNoDelay());
}

void KickEngineNode::publishKick()
{
	// TODO testing
	// TODO cleanup

	if (m_p_node_service->convertGoalCoordinateFromSupportFootToTrunkBased())
	{
		publishControlerCommands();
	}

	m_ros_publisher_support.publish(m_p_node_service->getSupportFootState());

	if (m_sp_debug_service->isDebugOn())
	{
		publishDebug();
		publishMarkers();
	}
}

void KickEngineNode::publishOdemetry()
{
	// TODO testing
	// TODO cleanup

	tf::Vector3 position;
	geometry_msgs::Quaternion quaternion_msg;
	m_p_node_service->getOdemetryData(position, quaternion_msg);

	ros::Time current_time = ros::Time::now();
	std::string frame_id = "odom";
	std::string child_frame_id = "base_link";

	// send the odometry as transformation
	geometry_msgs::TransformStamped odometry_transformation = geometry_msgs::TransformStamped();

	odometry_transformation.header.stamp = current_time;
	odometry_transformation.header.frame_id = frame_id;
	odometry_transformation.child_frame_id = child_frame_id;
	odometry_transformation.transform.translation.x = position[0];
	odometry_transformation.transform.translation.y = position[1];
	odometry_transformation.transform.translation.z = position[2];
	odometry_transformation.transform.rotation = quaternion_msg;

	tf::TransformBroadcaster odometry_broadcaster;
	odometry_broadcaster.sendTransform(odometry_transformation);

	// send the odometry also as message
	nav_msgs::Odometry odometry_nav_msgs;

	odometry_nav_msgs.header.stamp = current_time;
	odometry_nav_msgs.header.frame_id = frame_id;
	odometry_nav_msgs.child_frame_id = child_frame_id;
	odometry_nav_msgs.pose.pose.position.x = position[0];
	odometry_nav_msgs.pose.pose.position.y = position[1];
	odometry_nav_msgs.pose.pose.position.z = position[2];
	odometry_nav_msgs.pose.pose.orientation = quaternion_msg;
	odometry_nav_msgs.twist.twist = m_p_node_service->getTwist();

	m_ros_publisher_odometry.publish(odometry_nav_msgs);
}

void KickEngineNode::publishControlerCommands()
{
	// TODO testing
	// TODO cleanup
	bitbots_msgs::JointCommand joint_command_msg;

	joint_command_msg.header.stamp = ros::Time::now();
	m_p_node_service->getGoalFeetJoints(joint_command_msg.positions, joint_command_msg.joint_names);

	std::vector<double> ones(joint_command_msg.joint_names.size(), -1.0);
	joint_command_msg.velocities = ones;
	joint_command_msg.accelerations = ones;
	joint_command_msg.max_currents = ones;

	m_ros_publisher_controller_command.publish(joint_command_msg);
}

/*
	This method publishes various debug / visualization information.
*/
void KickEngineNode::publishDebug()
{
	// TODO testing
	// TODO cleanup

	// define frames
	std::string frame_base_link = "base_link";
	std::string current_support_frame = m_sp_debug_service->getSupportFootSole();

	// define colors
	std_msgs::ColorRGBA left_feet_color = m_p_node_service->createColorRGBA(0, 1, 0, 1);
	std_msgs::ColorRGBA right_feet_color = m_p_node_service->createColorRGBA(1, 0, 0, 1);
	std_msgs::ColorRGBA fly_feet_color = m_p_node_service->createColorRGBA(0, 0, 1, 1);
	std_msgs::ColorRGBA support_feet_color = m_p_node_service->createColorRGBA(1, 1, 0, 1);

	if (m_sp_debug_service->areBoothFeetSupport())
	{
		support_feet_color = m_p_node_service->createColorRGBA(0, 0, 1, 1);
	}
	else if (m_sp_debug_service->isLeftFootSupport())
	{
		support_feet_color = m_p_node_service->createColorRGBA(1, 0, 0, 1);
	}

	auto msg = createDebugMessage();

	// engine output
	publishMarker("engine_fly_goal", current_support_frame, msg.engine_fly_goal, fly_feet_color);
	publishMarker("engine_trunk_goal", current_support_frame, msg.engine_trunk_goal, support_feet_color);

	// resulting trunk pose
	publishMarker("trunk_result", frame_base_link, m_p_node_service->getTrunkResult(), support_feet_color);

	// goals
	publishMarker("engine_left_goal", frame_base_link, msg.left_foot_goal, left_feet_color);
	publishMarker("engine_right_goal", frame_base_link, msg.right_foot_goal, right_feet_color);

	// IK results
	publishMarker("ik_left", frame_base_link, msg.left_foot_ik_result, left_feet_color);
	publishMarker("ik_right", frame_base_link, msg.right_foot_ik_result, right_feet_color);

	m_ros_publisher_debug.publish(msg);
}

void KickEngineNode::publishMarkers()
{
	// TODO testing
	// TODO cleanup

	//publish markers
	visualization_msgs::Marker marker_msg;
	auto step_scale = m_p_node_service->createVector3(0.20, 0.10, 0.01);

	marker_msg.header.stamp = ros::Time::now();
	marker_msg.header.frame_id = m_p_node_service->getSupportFootSole();
	marker_msg.type = marker_msg.CUBE;
	marker_msg.action = marker_msg.ADD;
	marker_msg.lifetime = ros::Duration(0.0);
	marker_msg.scale = step_scale;

	//last step
	marker_msg.ns = "last_step";
	marker_msg.id = 1;
	marker_msg.color = m_p_node_service->createColorRGBA(0, 0, 0, 1);
	marker_msg.pose = m_p_node_service->getLastFootstepPose();
	m_ros_publisher_debug_marker.publish(marker_msg);

	//last step center
	marker_msg.ns = "step_center";
	marker_msg.id = m_int_marker_id;
	marker_msg.scale = m_p_node_service->createVector3(0.01, 0.01, 0.01);
	m_ros_publisher_debug_marker.publish(marker_msg);

	// next step
	marker_msg.id = m_int_marker_id;
	marker_msg.ns = "next_step";
	marker_msg.scale = step_scale;
	marker_msg.color = m_p_node_service->createColorRGBA(1, 1, 1, 0.5);
	marker_msg.pose = m_p_node_service->getNextFootstepPose();
	m_ros_publisher_debug_marker.publish(marker_msg);

	m_int_marker_id++;
}

void KickEngineNode::publishMarker(std::string name_space, std::string frame, geometry_msgs::Pose pose, std_msgs::ColorRGBA color)
{
	// TODO testing
	// TODO cleanup

	visualization_msgs::Marker marker_msg;

	marker_msg.header.stamp = ros::Time::now();
	marker_msg.header.frame_id = frame;
	marker_msg.type = marker_msg.ARROW;
	marker_msg.ns = name_space;
	marker_msg.action = marker_msg.ADD;
	marker_msg.pose = pose;
	marker_msg.color = color;
	marker_msg.scale = m_p_node_service->createVector3(0.01, 0.003, 0.003);
	marker_msg.id = m_int_marker_id;

	m_ros_publisher_debug_marker.publish(marker_msg);
	m_int_marker_id++;
}

bitbots_kick_engine::WalkingDebug KickEngineNode::createDebugMessage()
{
	// TODO testing
	// TODO cleanup

	bitbots_kick_engine::WalkingDebug msg;

	msg.is_left_support = m_p_node_service->isLeftFootSupport();
	msg.is_double_support = m_p_node_service->areBoothFeetSupport();
	msg.header.stamp = ros::Time::now();

	// times
	msg.phase_time = m_sp_debug_service->getEnginePhaseTime();
	msg.traj_time = m_sp_debug_service->getTrajectoryTime();

	// engine output
	msg.engine_fly_goal = m_sp_debug_service->getEngineFlyFootGoalPose();
	msg.engine_trunk_goal = m_sp_debug_service->getEngineTrunkGoalPose();

	if (m_sp_debug_service->calculateDebugData())
	{
		// goals
		msg.left_foot_goal = m_sp_debug_service->getGoalLeftFoot();
		msg.right_foot_goal = m_sp_debug_service->getGoalRightFoot();
		msg.fly_foot_goal = m_sp_debug_service->getGoalFlyFoot();
		msg.support_foot_goal = m_sp_debug_service->getGoalSupportFoot();

		// IK results
		msg.left_foot_ik_result = m_sp_debug_service->getIkResultLeftFoot();
		msg.right_foot_ik_result = m_sp_debug_service->getIkResultRightFoot();
		msg.fly_foot_ik_result = m_sp_debug_service->getIkResultFlyFoot();
		msg.support_foot_ik_result = m_sp_debug_service->getIkResultSupportFoot();

		// IK offsets
		msg.left_foot_ik_offset = m_sp_debug_service->getOffsetIkLeftFoot();
		msg.right_foot_ik_offset = m_sp_debug_service->getOffsetIkRightFoot();
		msg.fly_foot_ik_offset = m_sp_debug_service->getOffsetIkFlyFoot();
		msg.support_foot_ik_offset = m_sp_debug_service->getOffsetIkSupportFoot();

		// actual positions
		msg.left_foot_position = m_sp_debug_service->getPositionLeftFoot();
		msg.right_foot_position = m_sp_debug_service->getPositionRightFoot();
		msg.fly_foot_position = m_sp_debug_service->getPositionFlyFoot();
		msg.support_foot_position = m_sp_debug_service->getPositionSupportFoot();

		// actual offsets
		msg.left_foot_actual_offset = m_sp_debug_service->getOffsetPositionLeftFoot();
		msg.right_foot_actual_offset = m_sp_debug_service->getOffsetPositionRightFoot();
		msg.fly_foot_actual_offset = m_sp_debug_service->getOffsetPositionFlyFoot();
		msg.support_foot_actual_offset = m_sp_debug_service->getOffsetPositionSupportFoot();
	}

	return msg;
}

int main(int argc, char **argv)
{
	// TODO implementation
	// TODO testing
	// TODO cleanup
}