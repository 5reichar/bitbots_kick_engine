#include "KickEngine.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>

KickEngine::KickEngine()
{
	robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
	robot_model_loader.loadKinematicsSolvers(
		kinematics_plugin_loader::KinematicsPluginLoaderPtr(
			new kinematics_plugin_loader::KinematicsPluginLoader()));

	m_kinematic_model = robot_model_loader.getModel();
}

void KickEngine::set_param()
{
    // TODO: implementation
}

void KickEngine::set_goal_state(std::vector<std::string> vec_joint_names, std::vector<double> vec_position)
{
	m_goal_state.reset(new robot_state::RobotState(m_kinematic_model));
	m_goal_state->setToDefaultValues();

	for (int i = 0; i < names_vec.size(); i++)
	{
		// besides its name, this method only changes a single joint position...
		m_goal_state->setJointPositions(vec_joint_names[i], &vec_position[i]);
	}
}

void KickEngine::set_robot_state(const humanoid_league_msgs::RobotControlState msg)
{
    // TODO: implementation
}

void KickEngine::kick(geometry_msgs::Vector3 & ball_position, geometry_msgs::Vector3 & target_position)
{
    // TODO: implementation
}

std::vector<std::string> KickEngine::get_joint_names() const
{
    // TODO: implementation
}

std::vector<double> KickEngine::get_joint_goals() const
{
    // TODO: implementation
}

geometry_msgs::Twist KickEngine::get_twist() const
{
    // TODO: implementation

    geometry_msgs::Twist twist;
    twist.linear.x = m_v3d_current_orders[0] * m_parameter.freq * 2;
    twist.linear.y = m_v3d_current_orders[1] * m_parameter.freq * 2;
    twist.angular.z = m_v3d_current_orders[2] * m_parameter.freq * 2;

    return twist;
}


robot_state::RobotStatePtr KickEngine::get_goal_state() const
{
    // TODO: implementation
}

bool KickEngine::has_new_goals() const
{
    // TODO: implementation
}

geometry_msgs::Vector3 KickEngine::get_kick_start_position()
{
    // TODO: implementation
}

void KickEngine::move_left_feet_to_position(geometry_msgs::Vector3 position)
{
    // TODO: implementation
}

void KickEngine::move_right_feet_to_position(geometry_msgs::Vector3 position)
{
    // TODO: implementation
}

void KickEngine::move_feet_to_position(geometry_msgs::Vector3 position)
{
    // TODO: implementation
}
