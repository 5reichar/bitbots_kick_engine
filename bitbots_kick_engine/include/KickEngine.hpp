#ifndef KICKENGINE_HPP
#define KICKENGINE_HPP

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
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
	~KickEngine();

	bool update(double delta_time);

	std::shared_ptr<KickEngineParameter> get_engine_parameter();
	std::shared_ptr<KickParameter> get_kick_parameter();
	void set_robot_state(uint8_t state);

	virtual void set_goal_state(std::shared_ptr<moveit::core::RobotState> &goal_state);
	virtual void set_goal_state(std::vector<std::string> vec_joint_names, std::vector<double> vec_position);

	geometry_msgs::Twist get_twist() const;
	std::string get_support_foot_sole() const;
	moveit::core::JointModelGroup &get_joint_model_group(std::string name);
	Eigen::Isometry3d get_goal_global_link_transform(std::string link_name) const;
	Eigen::Isometry3d get_current_global_link_transform(std::string link_name) const;
	void get_goal_joint_group(std::string joint_group_name, std::vector<double> &joint_goals_out, std::vector<std::string> &joint_names_out);

	Eigen::Vector3d get_trunk_axis() const;
	Eigen::Vector3d get_fly_foot_axis() const;
	Eigen::Vector3d get_trunk_position() const;
	Eigen::Vector3d get_last_foot_step() const;
	Eigen::Vector3d get_next_foot_step() const;
	Eigen::Vector3d get_fly_foot_position() const;

	bool is_left_foot_support() const;
	bool are_booth_feet_support() const;

	void reset_current_state();
	double calc_trajectory_time() const;
	double get_engine_phase_time() const;
	bool kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position);

private:
	bool kick(struct3d* ball_position, struct3d* target_position, struct3d* foot_final_position = nullptr);
	void update_phase(double delta_time);

	double m_d_time_phase;
	uint8_t m_robot_state;
	KickFactory m_kick_factory;

	Footstep * m_p_footstep;
	KickFactory * m_p_kick_factory;

	std::shared_ptr<moveit::core::RobotState> m_sp_goal_state;
	std::shared_ptr<moveit::core::RobotState> m_sp_current_state;
	std::shared_ptr<moveit::core::RobotModel> m_sp_kinematic_model;
	std::shared_ptr<bitbots_splines::SplineContainer> m_sp_spline_container;

	// Configuration Parameter
	std::shared_ptr<KickEngineParameter> m_sp_kick_engine_parameter;

};

#endif