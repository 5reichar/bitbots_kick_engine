#ifndef KICKENGINENODESERVICE_HPP
#define KICKENGINENODESERVICE_HPP

#include "KickEngine.hpp"
#include <geometry_msgs/Vector3.h>
#include "bitbots_ik/BioIKSolver.hpp"
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Char.h>
#include "KickEngineDebugService.hpp"
#include <humanoid_league_msgs/RobotControlState.h>
#include <bitbots_kick_engine/kick_engine_paramsConfig.h>
#include <bitbots_kick_engine/kick_paramsConfig.h>

class KickEngineNodeService
{
public:
	KickEngineNodeService(bool simulation);

	bool convert_goal_coordinate_from_support_foot_to_trunk_based();
	bool kick(geometry_msgs::Vector3 &ball_position, geometry_msgs::Vector3 &target_position);
	double calculate_time_delta();
	void reconfigure_engine_parameter(bitbots_kick_engine::kick_engine_paramsConfig &config, uint32_t level);
	void reconfigure_kick_parameter(bitbots_kick_engine::kick_paramsConfig& config, uint32_t level);

	geometry_msgs::Vector3 create_vector_3(float x, float y, float z);
	std_msgs::ColorRGBA create_color_rgba(float red, float green, float blue, float alpha);

	bool is_left_foot_support();
	bool are_booth_feet_support();

	void set_robot_state(const humanoid_league_msgs::RobotControlState msg);

	std::string get_support_foot_sole() const;
	geometry_msgs::Twist get_twist() const;
	double get_engine_frequence() const;
	geometry_msgs::Pose get_trunk_result();
	std_msgs::Char get_support_foot_state();
	std::shared_ptr<KickEngineDebugService> get_debug_service();
	void get_odemetry_data(tf::Vector3 &position_out, geometry_msgs::Quaternion &quaternion_msg_out);
	void get_goal_feet_joints(std::vector<double> &joint_goals_out, std::vector<std::string> &joint_names_out);

	geometry_msgs::Pose get_last_footstep_pose();
	geometry_msgs::Pose get_next_footstep_pose();

private:
	tf::Transform get_support_foot_transformation(Eigen::Vector3d position, Eigen::Vector3d axis);


	bool m_b_first_run;
	bool m_b_simulation_active;
	double m_d_ros_time_last_update;
	std::chrono::time_point<std::chrono::steady_clock> m_time_point_last_update;

	geometry_msgs::Pose get_pose_from_step(Eigen::Vector3d step_position);

	std::shared_ptr<KickEngine> m_sp_kick_engine;
	bitbots_ik::BioIKSolver m_bio_ik_solver;
	std::shared_ptr<KickEngineDebugService> m_sp_debug_service;
};

#endif