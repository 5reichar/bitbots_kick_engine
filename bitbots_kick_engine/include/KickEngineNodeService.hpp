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
	~KickEngineNodeService();

	void set_debug(bool debug);

	bool kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position);

	bool is_left_foot_support();
	bool are_booth_feet_support();
	geometry_msgs::Pose get_last_footstep_pose();
	geometry_msgs::Pose get_next_footstep_pose();
	geometry_msgs::Pose get_engine_fly_foot_goal_pose();
	geometry_msgs::Pose get_engine_trunk_goal_pose();
	std_msgs::Char get_support_foot_sole();
	bool get_goal_feet_joints(std::vector<double>& joint_goals_out, std::vector<std::string>& joint_names_out);
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

	void get_feet_position(Eigen::Isometry3d (KickEngine::*get_global_link_transform) (std::string link_name), geometry_msgs::Pose& left_foot_out, geometry_msgs::Pose& right_foot_out, geometry_msgs::Pose& fly_foot_out, geometry_msgs::Pose& support_foot_out);
	void get_feet_offset(Eigen::Isometry3d(KickEngine::* get_global_link_transform) (std::string link_name), geometry_msgs::Vector3& left_foot_offset_out, geometry_msgs::Vector3& right_foot_offset_out, geometry_msgs::Vector3& fly_foot_offset_out, geometry_msgs::Vector3& support_foot_offset_out);

private:

	Eigen::Vector3d get_last_footstep();
	Eigen::Vector3d get_next_footstep();
	geometry_msgs::Pose get_pose(Eigen::Vector3d position, Eigen::Vector3d axis);
	geometry_msgs::Pose get_pose_from_step(Eigen::Vector3d step_position);
	tf::Transform get_support_foot_transformation(Eigen::Vector3d position, Eigen::Vector3d axis);

	KickEngine m_kick_engine;
    bitbots_ik::BioIKSolver m_bio_ik_solver;

	class KENSdebug
	{
	public:
		KENSdebug();
		~KENSdebug();

		tf::Transform m_tf_trunk_to_support_foot_goal;
		tf::Transform m_tf_trunk_to_flying_foot_goal;
	};

	KENSdebug* m_debug;
};

#endif