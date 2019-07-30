#include "KickEngine.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <bitbots_spline/include/utils/TrajectoryService.hpp>

KickEngine::KickEngine()
{
	//TODO: testing
	//TODO: cleanup

	robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
	robot_model_loader.loadKinematicsSolvers(
		kinematics_plugin_loader::KinematicsPluginLoaderPtr(
			new kinematics_plugin_loader::KinematicsPluginLoader()));

	this->m_sp_kinematic_model = robot_model_loader.getModel();
}

void KickEngine::set_parameter(std::shared_ptr<KickEngineParameter> &parameter)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_kick_engine_parameter = parameter;
}

void KickEngine::set_robot_state(uint8_t state)
{
	//TODO: testing
	//TODO: cleanup

	m_robot_state = state;
}

void KickEngine::set_goal_state(std::shared_ptr<moveit::core::RobotState> &goal_state)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_goal_state = goal_state;
}

void KickEngine::set_goal_state(std::vector<std::string> vec_joint_names, std::vector<double> vec_position)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_goal_state.reset(new robot_state::RobotState(m_kinematic_model));
	m_sp_goal_state->setToDefaultValues();

	for (int i = 0; i < vec_joint_names.size(); i++)
	{
		// besides its name, this method only changes a single joint position...
		m_sp_goal_state->setJointPositions(vec_joint_names[i], &vec_position[i]);
	}
}

double KickEngine::get_phase_time() const
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	return 0.0;
}

std::string KickEngine::get_state() const
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	return std::string();
}

std::shared_ptr<moveit::core::RobotState>& KickEngine::get_goal_state() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_goal_state;
}

geometry_msgs::Twist KickEngine::get_twist() const
{
	//TODO: testing
	//TODO: cleanup

	geometry_msgs::Twist twist;

	twist.linear.x = m_v3d_current_orders[0] * m_sp_kick_engine_parameter->freq * 2;
	twist.linear.y = m_v3d_current_orders[1] * m_sp_kick_engine_parameter->freq * 2;
	twist.angular.z = m_v3d_current_orders[2] * m_sp_kick_engine_parameter->freq * 2;

	return twist;
}

moveit::core::JointModelGroup& KickEngine::get_joint_model_group(std::string name)
{
	//TODO: testing
	//TODO: cleanup

	return *m_sp_kinematic_model->getJointModelGroup(name);
}

Eigen::Isometry3d KickEngine::get_goal_global_link_transform(std::string link_name) const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_goal_state->getGlobalLinkTransform(link_name);
}

Eigen::Isometry3d KickEngine::get_current_global_link_transform(std::string link_name) const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_current_state->getGlobalLinkTransform(link_name);
}

void KickEngine::get_goal_joint_group(std::string joint_group_name, std::vector<double>& joint_goals_out, std::vector<std::string>& joint_names_out)
{
	//TODO: testing
	//TODO: cleanup

	auto legs_joints_group = get_joint_model_group(joint_group_name);

	joint_names_out = legs_joints_group->getActiveJointModelNames();
	m_sp_goal_state->copyJointGroupPositions(legs_joints_group, joint_goals_out);
}

Eigen::Vector3d KickEngine::get_trunk_axis() const
{
	//TODO: testing
	//TODO: cleanup

	return bitbots_splines::TrajectoryService::GetTrajectorieAxisTrunk(calc_trajectory_time(), m_spline_container);
}

Eigen::Vector3d KickEngine::get_fly_foot_axis() const
{
	//TODO: testing
	//TODO: cleanup

	return bitbots_splines::TrajectoryService::GetTrajectorieAxisFoot(calc_trajectory_time(), m_spline_container);
}

Eigen::Vector3d KickEngine::get_trunk_position() const
{
	//TODO: testing
	//TODO: cleanup

	return bitbots_splines::TrajectoryService::GetTrajectoriePositionTrunk(calc_trajectory_time(), m_spline_container);
}

Eigen::Vector3d KickEngine::get_next_foot_step() const
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	if (is_left_foot_support())
	{
		x = _walkEngine.getFootstep().getLeft()[0];
		y = _walkEngine.getFootstep().getLeft()[1] + m_sp_kick_engine_parameter->footDistance / 2;
		yaw = _walkEngine.getFootstep().getLeft()[2];
	}
	else
	{
		x = _walkEngine.getFootstep().getRight()[0];
		y = _walkEngine.getFootstep().getRight()[1] + m_sp_kick_engine_parameter->footDistance / 2;
		yaw = _walkEngine.getFootstep().getRight()[2];
	}

	return Eigen::Vector3d(x, y, yaw);
}

Eigen::Vector3d KickEngine::get_last_foot_step() const
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	return Eigen::Vector3d();
}

Eigen::Vector3d KickEngine::get_fly_foot_position() const
{
	//TODO: testing
	//TODO: cleanup

	return bitbots_splines::TrajectoryService::GetTrajectoriePositionFoot(calc_trajectory_time(), m_spline_container);
}

bool KickEngine::is_left_foot_support() const
{
	//TODO: testing
	//TODO: cleanup

	// returns true if the value of the "is_left_support_foot" spline is currently higher than 0.5
	// the spline should only have values of 0 or 1
	return bitbots_splines::TrajectoryService::GetTrajectorieFootSupportLeft(calc_trajectory_time(), m_spline_container);
}

bool KickEngine::are_booth_feet_support() const
{
	//TODO: testing
	//TODO: cleanup

	// returns true if the value of the "is_double_support" spline is currently higher than 0.5
	// the spline should only have values of 0 or 1
	return bitbots_splines::TrajectoryService::GetTrajectorieFootSupportDouble(calc_trajectory_time(), m_spline_container);
}

void KickEngine::reset_current_state()
{
	//TODO: testing
	//TODO: cleanup

	m_sp_current_state.reset(new robot_state::RobotState(m_kinematic_model));
	m_sp_current_state->setToDefaultValues();
}

double KickEngine::calc_trajectory_time() const
{
	//TODO: testing
	//TODO: cleanup

	double t;
	if (_phase < 0.5) {
		t = _phase / m_sp_kick_engine_parameter->freq;
	}
	else {
		t = (_phase - 0.5) / m_sp_kick_engine_parameter->freq;
	}

	return t;
}

void KickEngine::kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position)
{
	//TODO: testing
	//TODO: cleanup

	struct3d ball = { ball_position.x, ball_position.y, ball_position.z };
	struct3d goal = { target_position.x, target_position.y, target_position.z };

	m_spline_container = m_kick_factory.make_kick_trajection(ball, goal);
}
