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
#include <bitbots_kick_engine/kick_preparation_positionsConfig.h>

class [[deprecated]] KickEngineNodeService
{
public:
	KickEngineNodeService(bool simulation);

	bool convertGoalCoordinateFromSupportFootToTrunkBased();
	bool kick(geometry_msgs::Vector3 &ball_position, geometry_msgs::Vector3 &target_position);
	double calculateTimeDelta();
	void reconfigureKickPreparationPositions(bitbots_kick_engine::kick_preparation_positionsConfig &config, uint32_t level);
	void reconfigureEngineParameter(bitbots_kick_engine::kick_engine_paramsConfig &config, uint32_t level);
	void reconfigureKickParameter(bitbots_kick_engine::kick_paramsConfig& config, uint32_t level);

	geometry_msgs::Vector3 createVector3(float x, float y, float z);
	std_msgs::ColorRGBA createColorRGBA(float red, float green, float blue, float alpha);

	bool isLeftFootSupport();
	bool areBoothFeetSupport();

	void setRobotState(const humanoid_league_msgs::RobotControlState msg);

	std::string getSupportFootSole() const;
	geometry_msgs::Twist getTwist() const;
	double getEngineFrequence() const;
	geometry_msgs::Pose getTrunkResult();
	std_msgs::Char getSupportFootState();
	std::shared_ptr<KickEngineDebugService> getDebugService();
	void getOdemetryData(tf::Vector3 &position_out, geometry_msgs::Quaternion &quaternion_msg_out);
	void getGoalFeetJoints(std::vector<double> &joint_goals_out, std::vector<std::string> &joint_names_out);

	geometry_msgs::Pose getLastFootstepPose();
	geometry_msgs::Pose getNextFootstepPose();

private:
	tf::Transform getSupportFootTransformation(Eigen::Vector3d position, Eigen::Vector3d axis);

	KickPreparationPosition makePosition(struct3d position, double robot_sector_min, double robot_sector_max, double ball_sector_min, double ball_sector_max);
	KickType makeKickType(KickTypeId id, bool active,  double robot_req_angle_min, double robot_req_angle_max, double ball_req_angle_min, double ball_req_angle_max);

	bool m_b_first_run;
	bool m_b_simulation_active;
	double m_d_ros_time_last_update;
	std::chrono::time_point<std::chrono::steady_clock> m_time_point_last_update;

	geometry_msgs::Pose getPoseFromStep(Eigen::Vector3d step_position);

	std::shared_ptr<KickEngine> m_sp_kick_engine;
	bitbots_ik::BioIKSolver m_bio_ik_solver;
	std::shared_ptr<KickEngineDebugService> m_sp_debug_service;
};

#endif