#ifndef KICKENGINEDEBUGSERVICE_HPP
#define KICKENGINEDEBUGSERVICE_HPP

#include "KickEngine.hpp"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Char.h>

class [[deprecated]] KickEngineDebugService
{
public:
	KickEngineDebugService(std::shared_ptr<KickEngine> kick_engine);

	bool isDebugOn() const;

	void setDebug(bool debug_on);
	void setTrunkToSupportFootGoal(tf::Transform goal);
	void setTrunkToFlyingFootGoal(tf::Transform goal);

	double getTrajectoryTime();
	double getEnginePhaseTime();
	std::string getSupportFootSole();
	bool isLeftFootSupport() const;
	bool areBoothFeetSupport() const;
	geometry_msgs::Pose getLastFootstepPose();
	geometry_msgs::Pose getNextFootstepPose();
	geometry_msgs::Pose getEngineTrunkGoalPose();
	geometry_msgs::Pose getEngineFlyFootGoalPose();

	geometry_msgs::Pose getGoalLeftFoot() const;
	geometry_msgs::Pose getGoalRightFoot() const;
	geometry_msgs::Pose getGoalFlyFoot() const;
	geometry_msgs::Pose getGoalSupportFoot() const;
	geometry_msgs::Pose getIkResultLeftFoot() const;
	geometry_msgs::Pose getIkResultRightFoot() const;
	geometry_msgs::Pose getIkResultFlyFoot() const;
	geometry_msgs::Pose getIkResultSupportFoot() const;
	geometry_msgs::Pose getPositionLeftFoot() const;
	geometry_msgs::Pose getPositionRightFoot() const;
	geometry_msgs::Pose getPositionFlyFoot() const;
	geometry_msgs::Pose getPositionSupportFoot() const;
	geometry_msgs::Vector3 getOffsetIkLeftFoot() const;
	geometry_msgs::Vector3 getOffsetIkRightFoot() const;
	geometry_msgs::Vector3 getOffsetIkFlyFoot() const;
	geometry_msgs::Vector3 getOffsetIkSupportFoot() const;
	geometry_msgs::Vector3 getOffsetPositionLeftFoot() const;
	geometry_msgs::Vector3 getOffsetPositionRightFoot() const;
	geometry_msgs::Vector3 getOffsetPositionFlyFoot() const;
	geometry_msgs::Vector3 getOffsetPositionSupportFoot() const;

	bool calculateDebugData();

private:
	void getFeetPosition(Eigen::Isometry3d& left_sole, Eigen::Isometry3d& right_sole, geometry_msgs::Pose &left_foot_out, geometry_msgs::Pose &right_foot_out, geometry_msgs::Pose &fly_foot_out, geometry_msgs::Pose &support_foot_out);
	void getFeetOffset(Eigen::Isometry3d& left_sole, Eigen::Isometry3d& right_sole, geometry_msgs::Vector3 &left_foot_offset_out, geometry_msgs::Vector3 &right_foot_offset_out, geometry_msgs::Vector3 &fly_foot_offset_out, geometry_msgs::Vector3 &support_foot_offset_out);

	geometry_msgs::Pose getPose(Eigen::Vector3d position, Eigen::Vector3d axis);
	geometry_msgs::Pose getPoseFromStep(Eigen::Vector3d step_position);

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