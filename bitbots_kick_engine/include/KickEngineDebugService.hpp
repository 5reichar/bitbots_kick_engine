#ifndef KICKENGINEDEBUGSERVICE_HPP
#define KICKENGINEDEBUGSERVICE_HPP

#include "KickEngine.hpp"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Char.h>

class KickEngineDebugService
{
public:
	KickEngineDebugService(std::shared_ptr<KickEngine> kick_engine);

	bool is_debug_on() const;

	void set_debug(bool debug_on);
	void set_trunk_to_support_foot_goal(tf::Transform goal);
	void set_trunk_to_flying_foot_goal(tf::Transform goal);

	double get_trajectory_time();
	double get_engine_phase_time();
	std::string get_support_foot_sole();
	bool is_left_foot_support() const;
	bool are_booth_feet_support() const;
	geometry_msgs::Pose get_last_footstep_pose();
	geometry_msgs::Pose get_next_footstep_pose();
	geometry_msgs::Pose get_engine_trunk_goal_pose();
	geometry_msgs::Pose get_engine_fly_foot_goal_pose();

	geometry_msgs::Pose get_goal_left_foot() const;
	geometry_msgs::Pose get_goal_right_foot() const;
	geometry_msgs::Pose get_goal_fly_foot() const;
	geometry_msgs::Pose get_goal_support_foot() const;
	geometry_msgs::Pose get_ik_result_left_foot() const;
	geometry_msgs::Pose get_ik_result_right_foot() const;
	geometry_msgs::Pose get_ik_result_fly_foot() const;
	geometry_msgs::Pose get_ik_result_support_foot() const;
	geometry_msgs::Pose get_position_left_foot() const;
	geometry_msgs::Pose get_position_right_foot() const;
	geometry_msgs::Pose get_position_fly_foot() const;
	geometry_msgs::Pose get_position_support_foot() const;
	geometry_msgs::Vector3 get_offset_ik_left_foot() const;
	geometry_msgs::Vector3 get_offset_ik_right_foot() const;
	geometry_msgs::Vector3 get_offset_ik_fly_foot() const;
	geometry_msgs::Vector3 get_offset_ik_support_foot() const;
	geometry_msgs::Vector3 get_offset_position_left_foot() const;
	geometry_msgs::Vector3 get_offset_position_right_foot() const;
	geometry_msgs::Vector3 get_offset_position_fly_foot() const;
	geometry_msgs::Vector3 get_offset_position_support_foot() const;

	bool calculate_debug_data();

private:
	void get_feet_position(Eigen::Isometry3d& left_sole, Eigen::Isometry3d& right_sole, geometry_msgs::Pose &left_foot_out, geometry_msgs::Pose &right_foot_out, geometry_msgs::Pose &fly_foot_out, geometry_msgs::Pose &support_foot_out);
	void get_feet_offset(Eigen::Isometry3d& left_sole, Eigen::Isometry3d& right_sole, geometry_msgs::Vector3 &left_foot_offset_out, geometry_msgs::Vector3 &right_foot_offset_out, geometry_msgs::Vector3 &fly_foot_offset_out, geometry_msgs::Vector3 &support_foot_offset_out);

	geometry_msgs::Pose get_pose(Eigen::Vector3d position, Eigen::Vector3d axis);
	geometry_msgs::Pose get_pose_from_step(Eigen::Vector3d step_position);

	bool m_b_debug_on;

	std::shared_ptr<KickEngine> m_sp_kick_engine;

	tf::Transform *m_p_tf_trunk_to_support_foot_goal;
	tf::Transform *m_p_tf_trunk_to_flying_foot_goal;

	geometry_msgs::Pose *m_p_pose_goal_left_foot;
	geometry_msgs::Pose *m_p_pose_goal_right_foot;
	geometry_msgs::Pose *m_p_pose_goal_fly_foot;
	geometry_msgs::Pose *m_p_pose_goal_support_foot;
	geometry_msgs::Pose *m_p_pose_ik_result_left_foot;
	geometry_msgs::Pose *m_p_pose_ik_result_right_foot;
	geometry_msgs::Pose *m_p_pose_ik_result_fly_foot;
	geometry_msgs::Pose *m_p_pose_ik_result_support_foot;
	geometry_msgs::Pose *m_p_pose_position_left_foot;
	geometry_msgs::Pose *m_p_pose_position_right_foot;
	geometry_msgs::Pose *m_p_pose_position_fly_foot;
	geometry_msgs::Pose *m_p_pose_position_support_foot;
	geometry_msgs::Vector3 *m_p_vector3_offset_ik_left_foot;
	geometry_msgs::Vector3 *m_p_vector3_offset_ik_right_foot;
	geometry_msgs::Vector3 *m_p_vector3_offset_ik_fly_foot;
	geometry_msgs::Vector3 *m_p_vector3_offset_ik_support_foot;
	geometry_msgs::Vector3 *m_p_vector3_offset_position_left_foot;
	geometry_msgs::Vector3 *m_p_vector3_offset_position_right_foot;
	geometry_msgs::Vector3 *m_p_vector3_offset_position_fly_foot;
	geometry_msgs::Vector3 *m_p_vector3_offset_position_support_foot;
};

#endif