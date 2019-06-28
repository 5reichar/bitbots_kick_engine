#ifndef KICKENGINENODESERVICE_HPP
#define KICKENGINENODESERVICE_HPP

#include "KickEngine.hpp"
#include <geometry_msgs/Vector3.h>
#include "bitbots_ik/BioIKSolver.hpp"
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Char.h>

class KickEngineNodeService
{
public:

	KickEngineNodeService();

	bool kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position);

    std::vector<double> get_joint_goals();
	std::vector<std::string> get_joint_names();
	std_msgs::Char get_support_foot_state();
    bool convert_goal_coordinate_from_support_foot_to_trunk_based();
    void get_odemetry_data(tf::Vector3 & position_out, geometry_msgs::Quaternion & quaternion_msg_out);

private:

	robot_model::RobotModelPtr m_kinematic_model;
	robot_state::RobotStatePtr m_current_state;

	KickEngine m_kick_engine;
    bitbots_ik::BioIKSolver m_bio_ik_solver;
};

#endif