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

	bool is_left_foot_support();
	bool are_booth_feet_support();
	geometry_msgs::Pose get_last_footstep_pose();
	geometry_msgs::Pose get_next_footstep_pose();
	geometry_msgs::Pose get_engine_fly_foot_goal_pose();
	geometry_msgs::Pose get_engine_trunk_goal_pose();
	std_msgs::Char get_support_foot_sole();
    std::vector<double> get_joint_goals();
	std::vector<std::string> get_joint_names();
	std_msgs::Char get_support_foot_state();
    bool convert_goal_coordinate_from_support_foot_to_trunk_based();
    void get_odemetry_data(tf::Vector3 & position_out, geometry_msgs::Quaternion & quaternion_msg_out);
	double get_phase_time();
	double get_trajectory_time();
	std_msgs::String get_engine_state();
	void get_feet_goals(geometry_msgs::Pose& left_foot_goal_out, geometry_msgs::Pose& right_foot_goal_out, geometry_msgs::Pose& fly_foot_goal_out, geometry_msgs::Pose& support_foot_goal_out);
	void get_feet_ik_results(geometry_msgs::Pose& left_foot_ik_result_out, geometry_msgs::Pose& right_foot_ik_result_out, geometry_msgs::Pose& fly_foot_ik_result_out, geometry_msgs::Pose& support_foot_ik_result_out);
	void get_feet_ik_offset(geometry_msgs::Vector3& left_foot_ik_offset_out, geometry_msgs::Vector3& right_foot_ik_offset_out, geometry_msgs::Vector3& fly_foot_ik_offset_out, geometry_msgs::Vector3& support_foot_ik_offset_out);
	void get_feet_position(geometry_msgs::Pose& left_foot_position_out, geometry_msgs::Pose& right_foot_position_out, geometry_msgs::Pose& fly_foot_position_out, geometry_msgs::Pose& support_foot_position_out);
	void get_feet_position_offset(geometry_msgs::Vector3& left_foot_position_offset_out, geometry_msgs::Vector3& right_foot_position_offset_out, geometry_msgs::Vector3& fly_foot_position_offset_out, geometry_msgs::Vector3& support_foot_position_offset_out);
	geometry_msgs::Pose get_trunk_result();

	void get_feet_position(robot_state::RobotStatePtr& state, geometry_msgs::Pose& left_foot_out, geometry_msgs::Pose& right_foot_out, geometry_msgs::Pose& fly_foot_out, geometry_msgs::Pose& support_foot_out);
	void get_feet_offset(robot_state::RobotStatePtr& state, geometry_msgs::Vector3& left_foot_offset_out, geometry_msgs::Vector3& right_foot_offset_out, geometry_msgs::Vector3& fly_foot_offset_out, geometry_msgs::Vector3& support_foot_offset_out);

private:
	Eigen::Vector3d get_last_footstep();
	Eigen::Vector3d get_next_footstep();
	geometry_msgs::Pose get_pose(Eigen::Vector3d position, Eigen::Vector3d axis);
	geometry_msgs::Pose get_step_pose(Eigen::Vector3d step_position);

	robot_model::RobotModelPtr m_kinematic_model;
	robot_state::RobotStatePtr m_current_state;

	KickEngine m_kick_engine;
    bitbots_ik::BioIKSolver m_bio_ik_solver;
};

#endif