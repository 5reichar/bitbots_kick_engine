#ifndef KICKENGINE_HPP
#define KICKENGINE_HPP

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "engine/KickEngineParameter.hpp"
#include "engine/KickFactory.hpp"
#include "Footstep.hpp"

class KickEngine
{
public:
	enum class KickType : uint16_t
	{
		standard = 0
	};

	KickEngine();

	bool update(double delta_time);

	void set_parameter(std::shared_ptr<KickEngineParameter> &parameter);
	void set_robot_state(uint8_t state);

	virtual void set_goal_state(std::shared_ptr<moveit::core::RobotState> &goal_state);
	virtual void set_goal_state(std::vector<std::string> vec_joint_names, std::vector<double> vec_position);

	geometry_msgs::Twist get_twist() const;
	moveit::core::JointModelGroup &get_joint_model_group(std::string name);
	Eigen::Isometry3d get_goal_global_link_transform(std::string link_name) const;
	Eigen::Isometry3d get_current_global_link_transform(std::string link_name) const;
	void get_goal_joint_group(std::string joint_group_name, std::vector<double> &joint_goals_out, std::vector<std::string> &joint_names_out);

	Eigen::Vector3d get_trunk_axis() const;
	Eigen::Vector3d get_fly_foot_axis() const;
	Eigen::Vector3d get_trunk_position() const;
	Eigen::Vector3d get_next_foot_step() const;
	Eigen::Vector3d get_fly_foot_position() const;

	bool is_left_foot_support() const;
	bool are_booth_feet_support() const;

	void reset_current_state();
	double calc_trajectory_time() const;
	void kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position);

private:
	void kick(struct3d& ball_position, struct3d& target_position, struct3d& foot_final_position);
	void update_phase(double delta_time);

	double m_d_time_phase;
	Footstep m_footstep;
	uint8_t m_robot_state;
	KickFactory m_kick_factory;
	struct3d m_s3d_foot_goal_position;
	bitbots_splines::SplineContainer m_spline_container;

	std::shared_ptr<moveit::core::RobotState> m_sp_goal_state;
	std::shared_ptr<moveit::core::RobotState> m_sp_current_state;
	std::shared_ptr<moveit::core::RobotModel> m_sp_kinematic_model;
	std::shared_ptr<KickEngineParameter> m_sp_kick_engine_parameter;

};

#endif