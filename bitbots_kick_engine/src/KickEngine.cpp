#include "KickEngine.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "../../bitbots_spline/include/utils/TrajectoryService.hpp"

KickEngine::KickEngine()
{
	//TODO: testing
	//TODO: cleanup

	m_sp_kick_engine_parameter = std::make_shared<KickEngineParameter>();
	m_up_kick_factory = std::make_unique<KickFactory>(m_sp_kick_engine_parameter);
	m_up_footstep = std::make_unique<Footstep>(m_sp_kick_engine_parameter->footDistance, is_left_foot_support());

	robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
	robot_model_loader.loadKinematicsSolvers(
		kinematics_plugin_loader::KinematicsPluginLoaderPtr(
			new kinematics_plugin_loader::KinematicsPluginLoader()));
	m_sp_kinematic_model = robot_model_loader.getModel();
}

bool KickEngine::update(double delta_time)
{
	bool b_succ = true;

	update_phase(delta_time);

	return b_succ;
}

std::shared_ptr<KickEngineParameter> KickEngine::get_engine_parameter()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine_parameter;
}

std::shared_ptr<KickParameter> KickEngine::get_kick_parameter()
{
	//TODO: testing
	//TODO: cleanup

	return m_up_kick_factory->get_kick_parameter();
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

	m_sp_goal_state.reset(new robot_state::RobotState(m_sp_kinematic_model));
	m_sp_goal_state->setToDefaultValues();

	for (int i = 0; i < vec_joint_names.size(); i++)
	{
		// besides its name, this method only changes a single joint position...
		m_sp_goal_state->setJointPositions(vec_joint_names[i], &vec_position[i]);
	}
}

geometry_msgs::Twist KickEngine::get_twist() const
{
	//TODO: testing
	//TODO: cleanup

	geometry_msgs::Twist twist;

	auto foot_goal_position = m_up_kick_factory->get_last_kicks_parameter().foot_ending_position;

	twist.linear.x = foot_goal_position.x * m_sp_kick_engine_parameter->freq * 2;
	twist.linear.y = foot_goal_position.y * m_sp_kick_engine_parameter->freq * 2;
	twist.angular.z = foot_goal_position.z * m_sp_kick_engine_parameter->freq * 2;

	return twist;
}

std::string KickEngine::get_support_foot_sole() const
{
	//TODO: testing
	//TODO: cleanup

	return is_left_foot_support() ? "l_sole" : "r_sole";
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

	joint_names_out = legs_joints_group.getActiveJointModelNames();
	m_sp_goal_state->copyJointGroupPositions(&legs_joints_group, joint_goals_out);
}

Eigen::Vector3d KickEngine::get_trunk_axis() const
{
	//TODO: testing
	//TODO: cleanup

	return bitbots_splines::TrajectoryService::GetTrajectorieAxisTrunk(calc_trajectory_time(), (*m_up_spline_container.get()));
}

Eigen::Vector3d KickEngine::get_fly_foot_axis() const
{
	//TODO: testing
	//TODO: cleanup

	return bitbots_splines::TrajectoryService::GetTrajectorieAxisFoot(calc_trajectory_time(), (*m_up_spline_container.get()));
}

Eigen::Vector3d KickEngine::get_trunk_position() const
{
	//TODO: testing
	//TODO: cleanup

	return bitbots_splines::TrajectoryService::GetTrajectoriePositionTrunk(calc_trajectory_time(), (*m_up_spline_container.get()));
}

Eigen::Vector3d KickEngine::get_last_foot_step() const
{
	//TODO: implementation
	//TODO: testing
	//TODO: cleanup

	return Eigen::Vector3d();
}

Eigen::Vector3d KickEngine::get_next_foot_step() const
{
	//TODO: testing
	//TODO: cleanup

	auto current_time = calc_trajectory_time() + m_sp_kick_engine_parameter->engineFrequency;
	double x, y, yaw;

	if (is_left_foot_support())
	{
		x = m_up_footstep.getLeft()[0];
		y = m_up_footstep.getLeft()[1] + m_sp_kick_engine_parameter->footDistance / 2;
		yaw = m_up_footstep.getLeft()[2];
	}
	else
	{
		x = m_up_footstep.getRight()[0];
		y = m_up_footstep.getRight()[1] + m_sp_kick_engine_parameter->footDistance / 2;
		yaw = m_up_footstep.getRight()[2];
	}

	return Eigen::Vector3d(x, y, yaw);
}

Eigen::Vector3d KickEngine::get_fly_foot_position() const
{
	//TODO: testing
	//TODO: cleanup

	return bitbots_splines::TrajectoryService::GetTrajectoriePositionFoot(calc_trajectory_time(), (*m_up_spline_container.get()));
}

bool KickEngine::is_left_foot_support() const
{
	//TODO: testing
	//TODO: cleanup

	// returns true if the value of the "is_left_support_foot" spline is currently higher than 0.5
	// the spline should only have values of 0 or 1
	return bitbots_splines::TrajectoryService::GetTrajectorieFootSupportLeft(calc_trajectory_time(), (*m_up_spline_container.get()));
}

bool KickEngine::are_booth_feet_support() const
{
	//TODO: testing
	//TODO: cleanup

	// returns true if the value of the "is_double_support" spline is currently higher than 0.5
	// the spline should only have values of 0 or 1
	return bitbots_splines::TrajectoryService::GetTrajectorieFootSupportDouble(calc_trajectory_time(), (*m_up_spline_container.get()));
}

void KickEngine::reset_current_state()
{
	//TODO: testing
	//TODO: cleanup

	m_sp_current_state.reset(new robot_state::RobotState(m_sp_kinematic_model));
	m_sp_current_state->setToDefaultValues();
}

double KickEngine::calc_trajectory_time() const
{
	//TODO: testing
	//TODO: cleanup

	double trajectory_time;
	if (m_d_time_phase < 0.5) {
		trajectory_time = m_d_time_phase / m_sp_kick_engine_parameter->freq;
	}
	else {
		trajectory_time = (m_d_time_phase - 0.5) / m_sp_kick_engine_parameter->freq;
	}

	return trajectory_time;
}

double KickEngine::get_engine_phase_time() const
{
	//TODO: testing
	//TODO: cleanup

	return m_d_time_phase;
}

bool KickEngine::kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position)
{
	//TODO: testing
	//TODO: cleanup

	struct3d ball = { ball_position.x, ball_position.y, ball_position.z };
	struct3d goal = { target_position.x, target_position.y, target_position.z };

	return kick(&ball, &goal);
}

bool KickEngine::kick(struct3d* ball_position, struct3d* target_position, struct3d* foot_final_position)
{
	//TODO: testing
	//TODO: cleanup

	if (foot_final_position == nullptr)
	{
		m_up_footstep.stepFromOrders(Eigen::Vector3d(target_position.x, target_position.y, target_position.z));
	}
	else
	{
		m_up_footstep.stepFromOrders(Eigen::Vector3d(foot_final_position.x, foot_final_position.y, foot_final_position.z));
	}

	m_up_spline_container.reset(m_kick_factory.make_kick_trajection(ball_position, target_position, foot_final_position));

	return m_up_spline_container;
}

void KickEngine::update_phase(double delta_time)
{
	//TODO: testing
	//TODO: cleanup

	//Check for negative time step
	if (delta_time <= 0.0) {
		if (delta_time == 0.0) { //sometimes happens due to rounding
			delta_time = 0.0001;
		}
		else {
			ROS_ERROR_THROTTLE(1, "QuinticWalk exception negative dt phase= %f dt= %f", m_d_time_phase, delta_time);
			return;
		}
	}
	//Check for too long dt
	if (delta_time > 0.25 / m_sp_kick_engine_parameter->freq) {
		ROS_ERROR_THROTTLE(1, "QuinticWalk error too long dt phase= %f dt= %f", m_d_time_phase, delta_time);
		return;
	}

	//Update the phase
	m_d_time_phase += delta_time * m_sp_kick_engine_parameter->freq;

	// reset to 0 if step complete
	if (m_d_time_phase > 1.0) {
		m_d_time_phase = 0.0;
	}
}
