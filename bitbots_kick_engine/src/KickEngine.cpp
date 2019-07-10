#include "KickEngine.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>

KickEngine::KickEngine()
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
	robot_model_loader.loadKinematicsSolvers(
		kinematics_plugin_loader::KinematicsPluginLoaderPtr(
			new kinematics_plugin_loader::KinematicsPluginLoader()));

	m_kinematic_model = robot_model_loader.getModel();
}

void set_goal_state(std::vector<std::string> vec_joint_names, std::vector<double> vec_position)
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	m_goal_state.reset(new robot_state::RobotState(m_kinematic_model));
	m_goal_state->setToDefaultValues();

	for (int i = 0; i < names_vec.size(); i++)
	{
		// besides its name, this method only changes a single joint position...
		m_goal_state->setJointPositions(vec_joint_names[i], &vec_position[i]);
	}
}

void reset_current_state()
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	m_current_state.reset(new robot_state::RobotState(m_kinematic_model));
	m_current_state->setToDefaultValues();
}

geometry_msgs::Twist get_twist() const
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

    geometry_msgs::Twist twist;
    twist.linear.x = m_v3d_current_orders[0] * m_parameter.freq * 2;
    twist.linear.y = m_v3d_current_orders[1] * m_parameter.freq * 2;
    twist.angular.z = m_v3d_current_orders[2] * m_parameter.freq * 2;

    return twist;
}

bool is_left_foot_support() const
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	_footstep.isLeftSupport()

	return false;
}

bool are_booth_feet_support() const
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	// returns true if the value of the "is_double_support" spline is currently higher than 0.5
	// the spline should only have values of 0 or 1
	_trajs.get("is_double_support").pos(getTrajsTime()) >= 0.5;

	return false;
}

robot_state::JointModelGroup& get_joint_model_group(std::string name)
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	return *m_kinematic_model->getJointModelGroup(name);
}

void get_goal_joint_group(std::string joint_group_name, std::vector<double>& joint_goals_out, std::vector<std::string>& joint_names_out)
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	auto legs_joints_group = m_kick_engine.get_joint_model_group(joint_group_name);

	joint_names_out = legs_joints_group->getActiveJointModelNames();
	_goal_state->copyJointGroupPositions(legs_joints_group, joint_goals_out);
}

Eigen::Isometry3d get_goal_global_link_transform(std::string link_name)
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	return m_goal_state->getGlobalLinkTransform(link_name);
}

void get_next_step(double& x_out, double& y_out, double& yaw_out)
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	if (_walkEngine.getFootstep().isLeftSupport()) {
		x = _walkEngine.getFootstep().getLeft()[0];
		y = _walkEngine.getFootstep().getLeft()[1] + _params.footDistance / 2;
		yaw = _walkEngine.getFootstep().getLeft()[2];
	}
	else {
		x = _walkEngine.getFootstep().getRight()[0];
		y = _walkEngine.getFootstep().getRight()[1] + _params.footDistance / 2;
		yaw = _walkEngine.getFootstep().getRight()[2];
	}
}

double get_trajectory_time() const
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	double t;
	if (_phase < 0.5) {
		t = _phase / _params.freq;
	}
	else {
		t = (_phase - 0.5) / _params.freq;
	}

	return t;

	return 0.0;
}
