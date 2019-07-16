#ifndef KICKENGINE_HPP
#define KICKENGINE_HPP

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "KickEngineParameter.hpp"

class KickEngine
{
public:
	enum class KickType : uint16_t
	{
		standard = 0
	};

	KickEngine();

	void set_parameter(KickEngineParameter parameter);
	void set_robot_state(uint8_t state);

	virtual void set_goal_state(robot_state::RobotStatePtr &goal_state);
	virtual void set_goal_state(std::vector<std::string> vec_joint_names, std::vector<double> vec_position);

	double get_phase_time() const;
	double get_engine_frequence() const;
	std::string get_state() const;
	moveit::core::RobotStatePtr get_goal_state() const;

	double get_foot_distance() const;
	geometry_msgs::Twist get_twist() const;
	moveit::core::JointModelGroup &get_joint_model_group(std::string name);
	Eigen::Isometry3d get_goal_global_link_transform(std::string link_name) const;
	Eigen::Isometry3d get_current_global_link_transform(std::string link_name) const;
	void get_goal_joint_group(std::string joint_group_name, std::vector<double> &joint_goals_out, std::vector<std::string> &joint_names_out);

	Eigen::Vector3d get_trunk_axis() const;
	Eigen::Vector3d get_fly_foot_axis() const;
	Eigen::Vector3d get_trunk_position() const;
	Eigen::Vector3d get_next_foot_step() const;
	Eigen::Vector3d get_last_foot_step() const;
	Eigen::Vector3d get_fly_foot_position() const;

	bool has_new_goals() const;
	bool is_left_foot_support() const;
	bool are_booth_feet_support() const;

	void reset_current_state();
	double calc_trajectory_time() const;
	void kick(geometry_msgs::Vector3 &ball_position, geometry_msgs::Vector3 &target_position);

private:
	geometry_msgs::Vector3 get_kick_start_position();

	void reset_goal_state();
	void reset_current_state(robot_state::RobotStatePtr &state);
	void move_feet_to_position(geometry_msgs::Vector3 position);
	void move_left_feet_to_position(geometry_msgs::Vector3 position);
	void move_right_feet_to_position(geometry_msgs::Vector3 position);

	uint8_t m_robot_state;
	Eigen::Vector3d m_v3d_current_orders;
	KickEngineParameter m_struc_parameter;
	std::shared_ptr<moveit::core::RobotState> m_sp_goal_state;
	std::shared_ptr<moveit::core::RobotState> m_sp_current_state;
	std::shared_ptr<moveit::core::RobotModel> m_sp_kinematic_model;
};

#endif