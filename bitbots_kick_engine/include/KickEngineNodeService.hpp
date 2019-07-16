#ifndef KICKENGINENODESERVICE_HPP
#define KICKENGINENODESERVICE_HPP

#include "KickEngine.hpp"
#include <geometry_msgs/Vector3.h>
#include "bitbots_ik/BioIKSolver.hpp"
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Char.h>
#include <bitbots_kick_engine/bitbots_quintic_walk_paramsConfig.h>
#include "KickEngineDebugService.hpp"

class KickEngineNodeService
{
public:
	KickEngineNodeService();

	bool kick(geometry_msgs::Vector3 &ball_position, geometry_msgs::Vector3 &target_position);
	bool convert_goal_coordinate_from_support_foot_to_trunk_based();
	void reconfigure_parameter(bitbots_kick_engine::bitbots_quintic_walk_paramsConfig &config, uint32_t level);
	bool is_left_foot_support();
	bool are_booth_feet_support();

	std::shared_ptr<KickEngineDebugService> get_debug_service();
	void set_robot_state(const humanoid_league_msgs::RobotControlState msg);

	double get_engine_frequence() const;
	geometry_msgs::Pose get_trunk_result();
	bool get_goal_feet_joints(std::vector<double> &joint_goals_out, std::vector<std::string> &joint_names_out);
	void get_odemetry_data(tf::Vector3 &position_out, geometry_msgs::Quaternion &quaternion_msg_out);
	std_msgs::Char get_support_foot_state();

private:
	tf::Transform get_support_foot_transformation(Eigen::Vector3d position, Eigen::Vector3d axis);

	KickEngine m_kick_engine;
	std::shared_ptr<KickEngineDebugService> m_sp_debug_service;
	bitbots_ik::BioIKSolver m_bio_ik_solver;
};

#endif