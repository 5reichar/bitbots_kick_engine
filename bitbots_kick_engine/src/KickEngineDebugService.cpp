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

bool KickEngineDebugService::is_debug_on() const
{
	//TODO: testing
	//TODO: cleanup

	return m_b_debug_on;
}

void KickEngineDebugService::set_debug(bool debug_on)
{
	//TODO: testing
	//TODO: cleanup

	m_b_debug_on = debug_on;
}

void KickEngineDebugService::set_trunk_to_support_foot_goal(tf::Transform goal)
{
	//TODO: testing
	//TODO: cleanup

	m_p_tf_trunk_to_support_foot_goal = &goal;
}

void KickEngineDebugService::set_trunk_to_flying_foot_goal(tf::Transform goal)
{
	//TODO: testing
	//TODO: cleanup

	m_p_tf_trunk_to_flying_foot_goal = &goal;
}

double KickEngineDebugService::get_trajectory_time()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->calc_trajectory_time();
}

double KickEngineDebugService::get_engine_phase_time()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->get_engine_phase_time();
}

std::string KickEngineDebugService::get_support_foot_sole()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->get_support_foot_sole();
}

bool KickEngineDebugService::is_left_foot_support() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->is_left_foot_support();
}

bool KickEngineDebugService::are_booth_feet_support() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->are_booth_feet_support();
}

geometry_msgs::Pose KickEngineDebugService::get_last_footstep_pose()
{
	//TODO: testing
	//TODO: cleanup

	return get_pose_from_step(m_sp_kick_engine->get_last_foot_step());
}

geometry_msgs::Pose KickEngineDebugService::get_next_footstep_pose()
{
	//TODO: testing
	//TODO: cleanup

	return get_pose_from_step(m_sp_kick_engine->get_next_foot_step());
}

geometry_msgs::Pose KickEngineDebugService::get_engine_trunk_goal_pose()
{
	//TODO: testing
	//TODO: cleanup

	return get_pose(m_sp_kick_engine->get_trunk_position(), m_sp_kick_engine->get_trunk_axis());
}

geometry_msgs::Pose KickEngineDebugService::get_engine_fly_foot_goal_pose()
{
	//TODO: testing
	//TODO: cleanup

	return get_pose(m_sp_kick_engine->get_fly_foot_position(), m_sp_kick_engine->get_fly_foot_axis());
}

geometry_msgs::Pose KickEngineDebugService::get_goal_left_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_goal_left_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_goal_left_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_goal_right_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_goal_right_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_goal_right_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_goal_fly_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_goal_fly_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_goal_fly_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_goal_support_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_goal_support_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_goal_support_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_ik_result_left_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_ik_result_left_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_ik_result_left_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_ik_result_right_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_ik_result_right_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_ik_result_right_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_ik_result_fly_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_ik_result_fly_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_ik_result_fly_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_ik_result_support_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_ik_result_support_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_ik_result_support_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_position_left_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_position_left_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_position_left_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_position_right_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_position_right_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_position_right_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_position_fly_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_position_fly_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_position_fly_foot;
}

geometry_msgs::Pose KickEngineDebugService::get_position_support_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_pose_position_support_foot == nullptr ? geometry_msgs::Pose() : *m_p_pose_position_support_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::get_offset_ik_left_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_ik_left_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_ik_left_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::get_offset_ik_right_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_ik_right_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_ik_right_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::get_offset_ik_fly_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_ik_fly_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_ik_fly_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::get_offset_ik_support_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_ik_support_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_ik_support_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::get_offset_position_left_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_position_left_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_position_left_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::get_offset_position_right_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_position_right_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_position_right_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::get_offset_position_fly_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_position_fly_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_position_fly_foot;
}

geometry_msgs::Vector3 KickEngineDebugService::get_offset_position_support_foot() const
{
	//TODO: testing
	//TODO: cleanup

	return m_p_vector3_offset_position_support_foot == nullptr ? geometry_msgs::Vector3() : *m_p_vector3_offset_position_support_foot;
}

bool KickEngineDebugService::calculate_debug_data()
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

		if (m_sp_kick_engine->is_left_foot_support())
		{
			m_p_pose_goal_left_foot = &pose_support_foot_goal;
			m_p_pose_goal_right_foot = &pose_fly_foot_goal;
		}
		else
		{
			m_p_pose_goal_left_foot = &pose_fly_foot_goal;
			m_p_pose_goal_right_foot = &pose_support_foot_goal;
		}

		auto left_sole = m_sp_kick_engine->get_goal_global_link_transform("l_sole");
		auto right_sole = m_sp_kick_engine->get_goal_global_link_transform("r_sole");

		get_feet_position(left_sole,
						  right_sole,
						  *m_p_pose_ik_result_left_foot,
						  *m_p_pose_ik_result_right_foot,
						  *m_p_pose_ik_result_fly_foot,
						  *m_p_pose_ik_result_support_foot);

		get_feet_offset(left_sole,
						right_sole,
						*m_p_vector3_offset_ik_left_foot,
						*m_p_vector3_offset_ik_right_foot,
						*m_p_vector3_offset_ik_fly_foot,
						*m_p_vector3_offset_ik_support_foot);


		left_sole = m_sp_kick_engine->get_current_global_link_transform("l_sole");
		right_sole = m_sp_kick_engine->get_current_global_link_transform("r_sole");

		get_feet_position(left_sole,
						  right_sole,
						  *m_p_pose_position_left_foot,
						  *m_p_pose_position_right_foot,
						  *m_p_pose_position_fly_foot,
						  *m_p_pose_position_support_foot);

		get_feet_offset(left_sole,
						right_sole,
						*m_p_vector3_offset_position_left_foot,
						*m_p_vector3_offset_position_right_foot,
						*m_p_vector3_offset_position_fly_foot,
						*m_p_vector3_offset_position_support_foot);

		success = true;
	}

	return success;
}

void KickEngineDebugService::get_feet_position(Eigen::Isometry3d& left_sole, Eigen::Isometry3d& right_sole, geometry_msgs::Pose& left_foot_out, geometry_msgs::Pose& right_foot_out, geometry_msgs::Pose& fly_foot_out, geometry_msgs::Pose& support_foot_out)
{
	//TODO: testing
	//TODO: cleanup

	geometry_msgs::Pose left_foot;
	tf::poseEigenToMsg(left_sole, left_foot);
	left_foot_out = left_foot;

	geometry_msgs::Pose right_foot;
	tf::poseEigenToMsg(right_sole, right_foot);
	right_foot_out = right_foot;

	if (m_sp_kick_engine->is_left_foot_support())
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

void KickEngineDebugService::get_feet_offset(Eigen::Isometry3d& left_sole, Eigen::Isometry3d& right_sole, geometry_msgs::Vector3 &left_foot_offset_out, geometry_msgs::Vector3 &right_foot_offset_out, geometry_msgs::Vector3 &fly_foot_offset_out, geometry_msgs::Vector3 &support_foot_offset_out)
{
	//TODO: testing
	//TODO: cleanup

	tf::Vector3 left_foot;
	tf::vectorEigenToTF(left_sole.translation(), left_foot);

	tf::Vector3 right_foot;
	tf::vectorEigenToTF(right_sole.translation(), right_foot);

	tf::Vector3 fly_off;
	tf::Vector3 support_off;
	if (m_sp_kick_engine->is_left_foot_support())
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

geometry_msgs::Pose KickEngineDebugService::get_pose(Eigen::Vector3d position, Eigen::Vector3d axis)
{
	//TODO: testing
	//TODO: cleanup

	geometry_msgs::Pose pose;

	tf::pointEigenToMsg(position, pose.position);
	pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(axis[0], axis[1], axis[2]);

	return pose;
}

geometry_msgs::Pose KickEngineDebugService::get_pose_from_step(Eigen::Vector3d step_position)
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
