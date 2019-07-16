#include "KickEngineNode.hpp"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <bitbots_msgs/JointCommand.h>
#include <bitbots_kick_engine/WalkingDebug.h>

KickEngineNode::KickEngineNode()
	: m_node_service(),
	  m_sp_debug_service(m_node_service.get_debug_service())
{
	// TODO testing
	// TODO cleanup

	m_int_marker_id = 1;

	initialise_ros_subcribtions();
	initialise_ros_publisher();
}

void KickEngineNode::initialise_ros_subcribtions()
{
	// TODO testing
	// TODO cleanup

	m_ros_subsciber_kick = m_ros_node_handle.subscribe("kick", 1, &KickEngineNode::kick_callback, this, ros::TransportHints().tcpNoDelay());
	m_ros_subsciber_robot_state = m_ros_node_handle.subscribe("robot_state", 1, &KickEngineNode::robot_state_callback, this, ros::TransportHints().tcpNoDelay());
}

void KickEngineNode::initialise_ros_publisher()
{
	// TODO testing
	// TODO cleanup

	m_ros_publisher_controller_command = m_ros_node_handle.advertise<bitbots_msgs::JointCommand>("kick_motor_goals", 1);
	m_ros_publisher_odometry = m_ros_node_handle.advertise<nav_msgs::Odometry>("kick_odometry", 1);
	m_ros_publisher_support = m_ros_node_handle.advertise<std_msgs::Char>("kick_support_state", 1);

	m_ros_publisher_debug = m_ros_node_handle.advertise<bitbots_kick_engine::WalkingDebug>("kick_debug", 1);
	m_ros_publisher_debug_marker = m_ros_node_handle.advertise<visualization_msgs::Marker>("kick_debug_marker", 1);
}

void KickEngineNode::robot_state_callback(const humanoid_league_msgs::RobotControlState msg)
{
	// TODO testing
	// TODO cleanup

	m_node_service.set_robot_state(msg);
}

void KickEngineNode::kick_callback(const humanoid_league_msgs::Kick action)
{
	// TODO testing
	// TODO cleanup

	kick_ball(action.ball_pos, action.target);
}

void KickEngineNode::reconfigure_callback(bitbots_kick_engine::bitbots_quintic_walk_paramsConfig &config, uint32_t level)
{
	// TODO testing
	// TODO cleanup

	m__sp_debug_service->set_debug(config.debugActive);
	m_uint_odometry_publish_factor = config.odomPubFactor;

	m_node_service.reconfigure_parameter(config, level);
}

void KickEngineNode::kick_ball(geometry_msgs::Vector3 &ball_position, geometry_msgs::Vector3 &target_position)
{
	// TODO testing
	// TODO cleanup

	uint16_t odometry_counter = 1;
	ros::Rate loopRate(m_node_service.get_engine_frequence());

	while (ros::ok())
	{
		if (m_node_service.kick(ball_position, target_position))
		{
			publish_kick();
		}

		if (odometry_counter > m_uint_odometry_publish_factor)
		{
			publish_odemetry();
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

void KickEngineNode::publish_kick()
{
	// TODO testing
	// TODO cleanup

	if (m_node_service.convert_goal_coordinate_from_support_foot_to_trunk_based())
	{
		publish_controler_commands();
	}

	m_ros_publisher_support.publish(m_node_service.get_support_foot_state())

		if (m__sp_debug_service->is_debug_on())
	{
		publish_debug();
		publish_markers();
	}
}

void KickEngineNode::publish_odemetry()
{
	// TODO testing
	// TODO cleanup

	tf::Vector3 position;
	geometry_msgs::Quaternion quaternion_msg;
	m_node_service.get_odemetry_data(position, quaternion_msg);

	ros::Time current_time = ros::Time::now();
	std::string frame_id = "odom";
	std::string child_frame_id = "base_link";

	// send the odometry as transformation
	geometry_msgs::TransformStamped odometry_transformation() = geometry_msgs::TransformStamped();

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
	odometry_nav_msgs.twist.twist = m_kick_engine.get_twist();

	m_ros_publisher_odometry.publish(_odom_msg);
}

void KickEngineNode::publish_controler_commands()
{
	// TODO testing
	// TODO cleanup

	bitbots_msgs::JointCommand joint_command_msg;
	std::vector<double> ones(joint_names.size(), -1.0);

	joint_command_msg.header.stamp = ros::Time::now();
	m_node_service.get_goal_feet_joints(joint_command_msg.joint_names, joint_command_msg.positions);
	joint_command_msg.velocities = ones;
	joint_command_msg.accelerations = ones;
	joint_command_msg.max_currents = ones;

	m_ros_publisher_controller_command.publish(joint_command_msg);
}

/*
	This method publishes various debug / visualization information.
*/
void KickEngineNode::publish_debug()
{
	// TODO testing
	// TODO cleanup

	// define frames
	std::string frame_base_link = "base_link";
	std::string current_support_frame = m_node_service.get_support_foot_sole();

	// define colors
	std_msgs::ColorRGBA left_feet_color = m_node_service.create_color_rgba(0, 1, 0, 1);
	std_msgs::ColorRGBA right_feet_color = m_node_service.create_color_rgba(1, 0, 0, 1);
	std_msgs::ColorRGBA fly_feet_color = m_node_service.create_color_rgba(0, 0, 1, 1);
	std_msgs::ColorRGBA support_feet_color = m_node_service.create_color_rgba(1, 1, 0, 1);

	if (m_node_service.are_booth_feet_support())
	{
		support_feet_color = m_node_service.create_color_rgba(0, 0, 1, 1);
	}
	else if (m_node_service.is_left_support())
	{
		support_feet_color = m_node_service.create_color_rgba(1, 0, 0, 1);
	}

	auto msg = create_debug_message();

	// engine output
	publish_marker("engine_fly_goal", current_support_frame, msg.engine_fly_goal, fly_feet_color);
	publish_marker("engine_trunk_goal", current_support_frame, msg.engine_trunk_goal, support_feet_color);

	// resulting trunk pose
	publish_marker("trunk_result", frame_base_link, m_node_service.get_trunk_result(), support_feet_color);

	// goals
	publish_marker("engine_left_goal", frame_base_link, msg.left_foot_goal, left_feet_color);
	publish_marker("engine_right_goal", frame_base_link, msg.right_foot_goal, right_feet_color);

	// IK results
	publish_marker("ik_left", frame_base_link, msg.left_foot_ik_result, left_feet_color);
	publish_marker("ik_right", frame_base_link, msg.right_foot_ik_result, right_feet_color);

	m_ros_publisher_debug.publish(msg);
}

void KickEngineNode::publish_markers()
{
	// TODO testing
	// TODO cleanup

	//publish markers
	visualization_msgs::Marker marker_msg;
	auto step_scale = m_node_service.create_vector_3(0.20, 0.10, 0.01);

	marker_msg.header.stamp = ros::Time::now();
	marker_msg.header.frame_id = m_node_service.get_support_foot_sole();
	marker_msg.type = marker_msg.CUBE;
	marker_msg.action = marker_msg.ADD;
	marker_msg.lifetime = ros::Duration(0.0);
	marker_msg.scale = step_scale;

	//last step
	marker_msg.ns = "last_step";
	marker_msg.id = 1;
	marker_msg.color = m_node_service.create_color_rgba(0, 0, 0, 1);
	marker_msg.pose = m_node_service.get_last_footstep_pose();
	m_ros_publisher_debug_marker.publish(marker_msg);

	//last step center
	marker_msg.ns = "step_center";
	marker_msg.id = m_int_marker_id;
	marker_msg.scale = m_node_service.create_vector_3(0.01, 0.01, 0.01);
	m_ros_publisher_debug_marker.publish(marker_msg);

	// next step
	marker_msg.id = m_int_marker_id;
	marker_msg.ns = "next_step";
	marker_msg.scale = step_scale;
	marker_msg.color = m_node_service.create_color_rgba(1, 1, 1, 0.5);
	marker_msg.pose = m_node_service.get_next_footstep_pose();
	m_ros_publisher_debug_marker.publish(marker_msg);

	m_int_marker_id++;
}

void KickEngineNode::publish_marker(std::string name_space, std::string frame, geometry_msgs::Pose pose, std_msgs::ColorRGBA color)
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
	marker_msg.scale = m_node_service.create_vector_3(0.01, 0.003, 0.003);
	marker_msg.id = m_int_marker_id;

	m_ros_publisher_debug_marker.publish(marker_msg);
	m_int_marker_id++;
}

bitbots_quintic_walk::WalkingDebug KickEngineNode::create_debug_message()
{
	bitbots_quintic_walk::WalkingDebug msg;

	msg.is_left_support = m_node_service.is_left_foot_support();
	msg.is_double_support = m_node_service.are_booth_feet_support();
	msg.header.stamp = ros::Time::now();

	// times
	msg.phase_time = m_sp_debug_service->get_engine_phase_time();
	msg.traj_time = m_sp_debug_service->get_engine_trajectory_time();

	msg.engine_state.data = m_sp_debug_service->get_engine_state();

	// engine output
	msg.engine_fly_goal = m_sp_debug_service->get_engine_fly_foot_goal_pose();
	msg.engine_trunk_goal = m_sp_debug_service->get_engine_trunk_goal_pose();

	// goals
	m_sp_debug_service->get_feet_goals(msg.left_foot_goal, msg.right_foot_goal, msg.fly_foot_goal, msg.support_foot_goal);

	// IK results
	m_sp_debug_service->get_feet_ik_results(msg.left_foot_ik_result, msg.right_foot_ik_result, msg.fly_foot_ik_result, support_foot_ik_result);

	// IK offsets
	m_sp_debug_service->get_feet_ik_offset(msg.left_foot_ik_offset, msg.right_foot_ik_offset, msg.fly_foot_ik_offset, msg.support_foot_ik_offset);

	// actual positions
	m_sp_debug_service->get_feet_position(msg.left_foot_position, msg.right_foot_position, msg.fly_foot_position, msg.support_foot_position);

	// actual offsets
	m_sp_debug_service->get_feet_position_offset(msg.left_foot_actual_offset, msg.right_foot_actual_offset, msg.fly_foot_actual_offset, msg.support_foot_actual_offset);

	return msg;
}

int main(int argc, char **argv)
{
	// TODO implementation
	// TODO testing
	// TODO cleanup
}