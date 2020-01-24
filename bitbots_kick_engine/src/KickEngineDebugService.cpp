#include "KickEngineDebugService.hpp"
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

KickEngineDebugService::KickEngineDebugService(std::shared_ptr<KickEngine> kick_engine)
	: m_sp_kick_engine(kick_engine)
{
	//TODO: testing
	//TODO: cleanup

	m_b_debug_on = false;
}

bool KickEngineDebugService::isDebugOn() const
{
	//TODO: testing
	//TODO: cleanup

	return m_b_debug_on;
}

void KickEngineDebugService::setDebug(bool debug_on)
{
	//TODO: testing
	//TODO: cleanup

	m_b_debug_on = debug_on;
}

void KickEngineDebugService::setTrunkToSupportFootGoal(tf::Transform goal)
{
	//TODO: testing
	//TODO: cleanup

	m_p_tf_trunk_to_support_foot_goal = &goal;
}

void KickEngineDebugService::setTrunkToFlyingFootGoal(tf::Transform goal)
{
	//TODO: testing
	//TODO: cleanup

	m_p_tf_trunk_to_flying_foot_goal = &goal;
}

double KickEngineDebugService::getTrajectoryTime()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->calcTrajectoryTime();
}

double KickEngineDebugService::getEnginePhaseTime()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->getEnginePhaseTime();
}

std::string KickEngineDebugService::getSupportFootSole()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->getSupportFootSole();
}

bool KickEngineDebugService::isLeftFootSupport() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->isLeftFootSupport();
}

bool KickEngineDebugService::areBoothFeetSupport() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->areBoothFeetSupport();
}

geometry_msgs::Pose KickEngineDebugService::getLastFootstepPose()
{
	//TODO: testing
	//TODO: cleanup

	return getPoseFromStep(m_sp_kick_engine->getLastFootStep());
}

geometry_msgs::Pose KickEngineDebugService::getNextFootstepPose()
{
	//TODO: testing
	//TODO: cleanup

	return getPoseFromStep(m_sp_kick_engine->getNextFootStep());
}

geometry_msgs::Pose KickEngineDebugService::getEngineTrunkGoalPose()
{
	//TODO: testing
	//TODO: cleanup

	return getPose(m_sp_kick_engine->getTrunkPosition(), m_sp_kick_engine->getTrunkAxis());
}

geometry_msgs::Pose KickEngineDebugService::getEngineFlyFootGoalPose()
{
	//TODO: testing
	//TODO: cleanup

	return getPose(m_sp_kick_engine->getFlyFootPosition(), m_sp_kick_engine->getFlyFootAxis());
}

geometry_msgs::Pose KickEngineDebugService::getGoalLeftFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_goal_left_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_goal_left_foot;
}

geometry_msgs::Pose KickEngineDebugService::getGoalRightFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_goal_right_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_goal_right_foot;
}

geometry_msgs::Pose KickEngineDebugService::getGoalFlyFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_goal_fly_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_goal_fly_foot;
}

geometry_msgs::Pose KickEngineDebugService::getGoalSupportFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_goal_support_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_goal_support_foot;
}

geometry_msgs::Pose KickEngineDebugService::getIkResultLeftFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_ik_result_left_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_ik_result_left_foot;
}

geometry_msgs::Pose KickEngineDebugService::getIkResultRightFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_ik_result_right_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_ik_result_right_foot;
}

geometry_msgs::Pose KickEngineDebugService::getIkResultFlyFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_ik_result_fly_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_ik_result_fly_foot;
}

geometry_msgs::Pose KickEngineDebugService::getIkResultSupportFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_ik_result_support_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_ik_result_support_foot;
}

geometry_msgs::Pose KickEngineDebugService::getPositionLeftFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_position_left_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_position_left_foot;
}

geometry_msgs::Pose KickEngineDebugService::getPositionRightFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_position_right_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_position_right_foot;
}

geometry_msgs::Pose KickEngineDebugService::getPositionFlyFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_position_fly_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_position_fly_foot;
}

geometry_msgs::Pose KickEngineDebugService::getPositionSupportFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_position_support_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_position_support_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::getOffsetIkLeftFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_ik_left_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_ik_left_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::getOffsetIkRightFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_ik_right_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_ik_right_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::getOffsetIkFlyFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_ik_fly_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_ik_fly_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::getOffsetIkSupportFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_ik_support_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_ik_support_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::getOffsetPositionLeftFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_position_left_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_position_left_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::getOffsetPositionRightFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_position_right_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_position_right_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::getOffsetPositionFlyFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_position_fly_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_position_fly_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::getOffsetPositionSupportFoot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_position_support_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_position_support_foot;
}

bool KickEngineDebugService::calculateDebugData()
{
	//TODO: testing
	//TODO: cleanup

	bool success = false;

	if (m_b_debug_on)
	{
		geometry_msgs::Pose pose_support_foot_goal;
		tf::pointTFToMsg(m_p_tf_trunk_to_support_foot_goal->getOrigin(), pose_support_foot_goal.position);
		tf::quaternionTFToMsg(m_p_tf_trunk_to_support_foot_goal->getRotation(), pose_support_foot_goal.orientation);
		m_p_pose_goal_support_foot = &pose_support_foot_goal;

		geometry_msgs::Pose pose_fly_foot_goal;
		tf::pointTFToMsg(m_p_tf_trunk_to_flying_foot_goal->getOrigin(), pose_fly_foot_goal.position);
		tf::quaternionTFToMsg(m_p_tf_trunk_to_flying_foot_goal->getRotation(), pose_fly_foot_goal.orientation);
		m_p_pose_goal_fly_foot = &pose_fly_foot_goal;

		if (m_sp_kick_engine->isLeftFootSupport())
		{
			m_p_pose_goal_left_foot = &pose_support_foot_goal;
			m_p_pose_goal_right_foot = &pose_fly_foot_goal;
		}
		else
		{
			m_p_pose_goal_left_foot = &pose_fly_foot_goal;
			m_p_pose_goal_right_foot = &pose_support_foot_goal;
		}

		auto left_sole = m_sp_kick_engine->getGoalGlobalLinkTransform("l_sole");
		auto right_sole = m_sp_kick_engine->getGoalGlobalLinkTransform("r_sole");

		getFeetPosition(left_sole,
						  right_sole,
						  *m_p_pose_ik_result_left_foot,
						  *m_p_pose_ik_result_right_foot,
						  *m_p_pose_ik_result_fly_foot,
						  *m_p_pose_ik_result_support_foot);

		getFeetOffset(left_sole,
						right_sole,
						*m_p_vector3_offset_ik_left_foot,
						*m_p_vector3_offset_ik_right_foot,
						*m_p_vector3_offset_ik_fly_foot,
						*m_p_vector3_offset_ik_support_foot);


		left_sole = m_sp_kick_engine->getCurrentGlobalLinkTransform("l_sole");
		right_sole = m_sp_kick_engine->getCurrentGlobalLinkTransform("r_sole");

		getFeetPosition(left_sole,
						  right_sole,
						  *m_p_pose_position_left_foot,
						  *m_p_pose_position_right_foot,
						  *m_p_pose_position_fly_foot,
						  *m_p_pose_position_support_foot);

		getFeetOffset(left_sole,
						right_sole,
						*m_p_vector3_offset_position_left_foot,
						*m_p_vector3_offset_position_right_foot,
						*m_p_vector3_offset_position_fly_foot,
						*m_p_vector3_offset_position_support_foot);

		success = true;
	}

	return success;
}

void KickEngineDebugService::getFeetPosition(Eigen::Isometry3d& left_sole, Eigen::Isometry3d& right_sole, geometry_msgs::Pose& left_foot_out, geometry_msgs::Pose& right_foot_out, geometry_msgs::Pose& fly_foot_out, geometry_msgs::Pose& support_foot_out)
{
	//TODO: testing
	//TODO: cleanup

	geometry_msgs::Pose left_foot;
	tf::poseEigenToMsg(left_sole, left_foot);
	left_foot_out = left_foot;

	geometry_msgs::Pose right_foot;
	tf::poseEigenToMsg(right_sole, right_foot);
	right_foot_out = right_foot;

	if (m_sp_kick_engine->isLeftFootSupport())
	{
		support_foot_out = left_foot;
		fly_foot_out = right_foot;
	}
	else
	{
		support_foot_out = right_foot;
		fly_foot_out = left_foot;
	}
}

void KickEngineDebugService::getFeetOffset(Eigen::Isometry3d& left_sole, Eigen::Isometry3d& right_sole, geometry_msgs::Vector3 &left_foot_offset_out, geometry_msgs::Vector3 &right_foot_offset_out, geometry_msgs::Vector3 &fly_foot_offset_out, geometry_msgs::Vector3 &support_foot_offset_out)
{
	//TODO: testing
	//TODO: cleanup

	tf::Vector3 left_foot;
	tf::vectorEigenToTF(left_sole.translation(), left_foot);

	tf::Vector3 right_foot;
	tf::vectorEigenToTF(right_sole.translation(), right_foot);

	tf::Vector3 fly_off;
	tf::Vector3 support_off;
	if (m_sp_kick_engine->isLeftFootSupport())
	{
		support_off = m_p_tf_trunk_to_support_foot_goal->getOrigin() - left_foot;
		fly_off = m_p_tf_trunk_to_flying_foot_goal->getOrigin() - right_foot;
		tf::vector3TFToMsg(support_off, left_foot_offset_out);
		tf::vector3TFToMsg(fly_off, right_foot_offset_out);
	}
	else
	{
		support_off = m_p_tf_trunk_to_support_foot_goal->getOrigin() - right_foot;
		fly_off = m_p_tf_trunk_to_flying_foot_goal->getOrigin() - left_foot;
		tf::vector3TFToMsg(fly_off, left_foot_offset_out);
		tf::vector3TFToMsg(support_off, right_foot_offset_out);
	}

	tf::vector3TFToMsg(support_off, support_foot_offset_out);
	tf::vector3TFToMsg(fly_off, fly_foot_offset_out);
}

geometry_msgs::Pose KickEngineDebugService::getPose(Eigen::Vector3d position, Eigen::Vector3d axis)
{
	//TODO: testing
	//TODO: cleanup

	geometry_msgs::Pose pose;

	tf::pointEigenToMsg(position, pose.position);
	pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(axis[0], axis[1], axis[2]);

	return pose;
}

geometry_msgs::Pose KickEngineDebugService::getPoseFromStep(Eigen::Vector3d step_position)
{
	//TODO: testing
	//TODO: cleanup

	geometry_msgs::Pose pose;

	pose.position.x = step_position[0];
	pose.position.y = step_position[1];
	pose.position.z = 0;

	pose.orientation = tf::createQuaternionMsgFromYaw(step_position[2]);

	return pose;
}
