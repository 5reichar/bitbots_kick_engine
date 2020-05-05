#ifndef KICKENGINE_HPP
#define KICKENGINE_HPP

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include "engine/KickFactory.hpp"

class [[deprecated]] KickEngine
{
public:
	enum class KickType : uint16_t
	{
		standard = 0
	};

	KickEngine();
	~KickEngine();

	bool update(double delta_time);

	std::shared_ptr<KickEngineParameter> getEngineParameter();
	std::shared_ptr<KickParameter> getKickParameter();
	void setRobotState(uint8_t state);

	virtual void setGoalState(std::shared_ptr<moveit::core::RobotState> &goal_state);
	virtual void setGoalState(std::vector<std::string> vec_joint_names, std::vector<double> vec_position);

	geometry_msgs::Twist getTwist() const;
	std::string getSupportFootSole() const;
	moveit::core::JointModelGroup &getJointModelGroup(std::string name);
	Eigen::Isometry3d getGoalGlobalLinkTransform(std::string link_name) const;
	Eigen::Isometry3d getCurrentGlobalLinkTransform(std::string link_name) const;
	void getGoalJointGroup(std::string joint_group_name, std::vector<double> &joint_goals_out, std::vector<std::string> &joint_names_out);

	Eigen::Vector3d getTrunkAxis() const;
	Eigen::Vector3d getFlyFootAxis() const;
	Eigen::Vector3d getTrunkPosition() const;
	Eigen::Vector3d getLastFootStep() const;
	Eigen::Vector3d getNextFootStep() const;
	Eigen::Vector3d getFlyFootPosition() const;

	bool isLeftFootSupport() const;
	bool areBoothFeetSupport() const;

	void resetCurrentState();
	double calcTrajectoryTime() const;
	double getEnginePhaseTime() const;
	bool kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position);

private:
	bool kick(struct3d* ball_position, struct3d* target_position, struct3d* foot_final_position = nullptr);
	void updatePhase(double delta_time);

	double m_d_time_phase;
	uint8_t m_robot_state;

	KickFactory * m_p_kick_factory;

	std::shared_ptr<Footstep> m_sp_footstep;
	std::shared_ptr<moveit::core::RobotState> m_sp_goal_state;
	std::shared_ptr<moveit::core::RobotState> m_sp_current_state;
	std::shared_ptr<moveit::core::RobotModel> m_sp_kinematic_model;
	std::shared_ptr<bitbots_splines::SplineContainer> m_sp_spline_container;

	// Configuration Parameter
	std::shared_ptr<KickEngineParameter> m_sp_kick_engine_parameter;

};

#endif