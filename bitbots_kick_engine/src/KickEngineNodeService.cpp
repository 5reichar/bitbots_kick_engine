#include "KickEngineNodeService.hpp"


KickEngineNodeService::KickEngineNodeService()
{
	//TODO: Implementation


	// we have to set some good initial position in the goal state, since we are using a gradient
	// based method. Otherwise, the first step will be not correct
	std::vector<std::string> names_vec = { "LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch" };
	std::vector<double> pos_vec = { 0.7, -1.0, -0.4, -0.7, 1.0, 0.4 };
	m_kick_engine.set_goal_state(names_vec, pos_vec);

	m_current_state.reset(new robot_state::RobotState(m_kinematic_model));
	m_current_state->setToDefaultValues();

	m_bio_ik_solver = bitbots_ik::BioIKSolver(*m_kinematic_model->getJointModelGroup("All"),
		*m_kinematic_model->getJointModelGroup("LeftLeg"),
		*m_kinematic_model->getJointModelGroup("RightLeg"));
	m_bio_ik_solver.set_use_approximate(true);
}

bool KickEngineNodeService::kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position)
{
	//TODO: Implementation
	return false;
}

std::vector<double> KickEngineNodeService::get_joint_goals()
{
    //TODO: Implementation
}

std::vector<std::string> KickEngineNodeService::get_joint_names()
{
	//TODO: Implementation
	return std::vector<std::string>();
}

std_msgs::Char KickEngineNodeService::get_support_foot_state()
{
	//TODO: Implementation
	return std_msgs::Char();
}

bool KickEngineNodeService::convert_goal_coordinate_from_support_foot_to_trunk_based()
{
	//TODO: Implementation
	return false;
}

void KickEngineNodeService::get_odemetry_data(tf::Vector3 & position_out, geometry_msgs::Quaternion & quaternion_msg_out)
{
    //TODO: Implementation
}
