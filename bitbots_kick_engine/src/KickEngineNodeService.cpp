#include "KickEngineNodeService.hpp"


KickEngineNodeService::KickEngineNodeService()
{
	//TODO: Implementation


	// we have to set some good initial position in the goal state, since we are using a gradient
	// based method. Otherwise, the first step will be not correct
	std::vector<std::string> names_vec = { "LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch" };
	std::vector<double> pos_vec = { 0.7, -1.0, -0.4, -0.7, 1.0, 0.4 };
	m_kick_engine.set_goal_state(names_vec, pos_vec);

	m_current_state.reset(new robot_state::RobotState(m_kinematic_model));
	m_current_state->setToDefaultValues();

	m_bio_ik_solver = bitbots_ik::BioIKSolver(*m_kinematic_model->getJointModelGroup("All"),
		*m_kinematic_model->getJointModelGroup("LeftLeg"),
		*m_kinematic_model->getJointModelGroup("RightLeg"));
	m_bio_ik_solver.set_use_approximate(true);
}

bool KickEngineNodeService::kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position)
{
	bool success = false;

	if (m_kick_engine.has_new_goals())
	{
		m_kick_engine.kick(ball_position, target_position);
		success = true;
	}

	return success;
}

bool KickEngineNodeService::is_left_foot_support()
{
	//TODO: Implementation
	return false;
}

bool KickEngineNodeService::are_booth_feet_support()
{
	//TODO: Implementation
	return false;
}

geometry_msgs::Pose KickEngineNodeService::get_last_footstep_pose()
{
	return get_step_pose(get_last_footstep());
}

geometry_msgs::Pose KickEngineNodeService::get_next_footstep_pose()
{
	return get_step_pose(get_next_footstep());
}

geometry_msgs::Pose KickEngineNodeService::get_engine_fly_foot_goal_pose()
{
	//TODO: Implementation

	return get_pose(_footPos, _footAxis);
}

geometry_msgs::Pose KickEngineNodeService::get_engine_trunk_goal_pose()
{
	//TODO: Implementation

	return get_pose(_trunkPos, _trunkAxis);
}

std_msgs::Char KickEngineNodeService::get_support_foot_sole()
{
	return is_left_foot_support() ? "l_sole" : "r_sole";
}

std::vector<double> KickEngineNodeService::get_joint_goals()
{
    //TODO: Implementation
	return std::vector<double>();
}

std::vector<std::string> KickEngineNodeService::get_joint_names()
{
	//TODO: Implementation
	return std::vector<std::string>();
}

std_msgs::Char KickEngineNodeService::get_support_foot_state()
{
	//TODO: Implementation

	std_msgs::Char support_foot_state;

	if (are_booth_feet_support()) {
		support_state.data = 'd';
	}
	else if (is_left_foot_support()) {
		support_state.data = 'l';
	}
	else {
		support_state.data = 'r';
	}

	return support_foot_state
}

bool KickEngineNodeService::convert_goal_coordinate_from_support_foot_to_trunk_based()
{
	//TODO: Implementation

	 // read the cartesian positions and orientations for trunk and fly foot
	m_kick_engine.computeCartesianPosition(_trunkPos, _trunkAxis, _footPos, _footAxis, _isLeftSupport);

	// change goals from support foot based coordinate system to trunk based coordinate system
	tf::Vector3 tf_vec;
	tf::vectorEigenToTF(_trunkPos, tf_vec);
	tf::Quaternion tf_quat = tf::Quaternion();
	tf_quat.setRPY(_trunkAxis[0], _trunkAxis[1], _trunkAxis[2]);
	tf_quat.normalize();
	tf::Transform support_foot_to_trunk(tf_quat, tf_vec);
	tf::Transform trunk_to_support_foot_goal = support_foot_to_trunk.inverse();

	tf::vectorEigenToTF(_footPos, tf_vec);
	tf_quat.setRPY(_footAxis[0], _footAxis[1], _footAxis[2]);
	tf_quat.normalize();
	tf::Transform support_to_flying_foot(tf_quat, tf_vec);
	tf::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * support_to_flying_foot;

	// call ik solver
	bool success = m_bio_ik_solver.solve(trunk_to_support_foot_goal, trunk_to_flying_foot_goal,
		_walkEngine.getFootstep().isLeftSupport(), _goal_state);

	return success;
}

void KickEngineNodeService::get_odemetry_data(tf::Vector3 & position_out, geometry_msgs::Quaternion & quaternion_msg_out)
{
    //TODO: Implementation
}

double KickEngineNodeService::get_phase_time()
{
	//TODO: Implementation
	return _walkEngine.getPhase();
}

double KickEngineNodeService::get_trajectory_time()
{
	//TODO: Implementation
	return _walkEngine.getTrajsTime();
}

std_msgs::String KickEngineNodeService::get_engine_state()
{
	//TODO: Implementation
	return _walkEngine.getState();
}

void KickEngineNodeService::get_feet_goals(geometry_msgs::Pose& left_foot_goal_out, geometry_msgs::Pose& right_foot_goal_out, geometry_msgs::Pose& fly_foot_goal_out, geometry_msgs::Pose& support_foot_goal_out)
{
	//TODO: Implementation
	geometry_msgs::Pose pose_support_foot_goal;
	tf::pointTFToMsg(trunk_to_support_foot_goal.getOrigin(), pose_support_foot_goal.position);
	tf::quaternionTFToMsg(trunk_to_support_foot_goal.getRotation(), pose_support_foot_goal.orientation);
	support_foot_goal_out = pose_support_foot_goal;
	geometry_msgs::Pose pose_fly_foot_goal;
	tf::pointTFToMsg(trunk_to_flying_foot_goal.getOrigin(), pose_fly_foot_goal.position);
	tf::quaternionTFToMsg(trunk_to_flying_foot_goal.getRotation(), pose_fly_foot_goal.orientation);
	fly_foot_goal_out = pose_fly_foot_goal;
	if (is_left_support) {
		left_foot_goal_out = pose_support_foot_goal;
		right_foot_goal_out = pose_fly_foot_goal;
	}
	else {
		left_foot_goal_out = pose_fly_foot_goal;
		right_foot_goal_out = pose_support_foot_goal;
	}
}

void KickEngineNodeService::get_feet_ik_results(geometry_msgs::Pose& left_foot_ik_result_out, geometry_msgs::Pose& right_foot_ik_result_out, geometry_msgs::Pose& fly_foot_ik_result_out, geometry_msgs::Pose& support_foot_ik_result_out)
{
	//TODO: Implementation
	geometry_msgs::Pose pose_left_result;
	tf::poseEigenToMsg(_goal_state->getGlobalLinkTransform("l_sole"), pose_left_result);
	left_foot_ik_result_out = pose_left_result;
	geometry_msgs::Pose pose_right_result;
	tf::poseEigenToMsg(_goal_state->getGlobalLinkTransform("r_sole"), pose_right_result);
	right_foot_ik_result_out = pose_right_result;
	if (is_left_support) {
		support_foot_ik_result_out = pose_left_result;
		fly_foot_ik_result_out = pose_right_result;
	}
	else {
		support_foot_ik_result_out = pose_right_result;
		fly_foot_ik_result_out = pose_left_result;
	}
}

void KickEngineNodeService::get_feet_ik_offset(geometry_msgs::Vector3& left_foot_ik_offset_out, geometry_msgs::Vector3& right_foot_ik_offset_out, geometry_msgs::Vector3& fly_foot_ik_offset_out, geometry_msgs::Vector3& support_foot_ik_offset_out)
{
	//TODO: Implementation
	tf::Vector3 support_off;
	tf::Vector3 fly_off;
	tf::Vector3 tf_vec_left;
	tf::Vector3 tf_vec_right;
	tf::vectorEigenToTF(_goal_state->getGlobalLinkTransform("l_sole").translation(), tf_vec_left);
	tf::vectorEigenToTF(_goal_state->getGlobalLinkTransform("r_sole").translation(), tf_vec_right);
	geometry_msgs::Vector3 vect_msg;
	if (is_left_support) {
		support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_left;
		fly_off = trunk_to_flying_foot_goal.getOrigin() - tf_vec_right;
		tf::vector3TFToMsg(support_off, vect_msg);
		left_foot_ik_offset_out = vect_msg;
		tf::vector3TFToMsg(fly_off, vect_msg);
		right_foot_ik_offset_out = vect_msg;
	}
	else {
		support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_right;
		fly_off = trunk_to_flying_foot_goal.getOrigin() - tf_vec_left;
		tf::vector3TFToMsg(fly_off, vect_msg);
		left_foot_ik_offset_out = vect_msg;
		tf::vector3TFToMsg(support_off, vect_msg);
		right_foot_ik_offset_out = vect_msg;
	}
	tf::vector3TFToMsg(support_off, vect_msg);
	support_foot_ik_offset_out = vect_msg;
	tf::vector3TFToMsg(fly_off, vect_msg);
	fly_foot_ik_offset_out = vect_msg;
}

void KickEngineNodeService::get_feet_position(geometry_msgs::Pose& left_foot_position_out, geometry_msgs::Pose& right_foot_position_out, geometry_msgs::Pose& fly_foot_position_out, geometry_msgs::Pose& support_foot_position_out)
{
	//TODO: Implementation
	geometry_msgs::Pose pose_left_actual;
	tf::poseEigenToMsg(_current_state->getGlobalLinkTransform("l_sole"), pose_left_actual);
	left_foot_position_out = pose_left_actual;
	geometry_msgs::Pose pose_right_actual;
	tf::poseEigenToMsg(_current_state->getGlobalLinkTransform("r_sole"), pose_right_actual);
	right_foot_position_out = pose_right_actual;
	if (is_left_support) {
		support_foot_position_out = pose_left_actual;
		fly_foot_position_out = pose_right_actual;
	}
	else {
		support_foot_position_out = pose_right_actual;
		fly_foot_position_out = pose_left_actual;
	}
}

void KickEngineNodeService::get_feet_position_offset(geometry_msgs::Vector3& left_foot_position_offset_out, geometry_msgs::Vector3& right_foot_position_offset_out, geometry_msgs::Vector3& fly_foot_position_offset_out, geometry_msgs::Vector3& support_foot_position_offset_out)
{
	//TODO: Implementation
	tf::Vector3 support_off;
	tf::Vector3 fly_off;
	tf::Vector3 tf_vec_left;
	tf::Vector3 tf_vec_right;
	tf::vectorEigenToTF(_current_state->getGlobalLinkTransform("l_sole").translation(), tf_vec_left);
	tf::vectorEigenToTF(_current_state->getGlobalLinkTransform("r_sole").translation(), tf_vec_right);
	if (is_left_support) {
		support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_left;
		fly_off = trunk_to_flying_foot_goal.getOrigin() - tf_vec_right;
		tf::vector3TFToMsg(support_off, vect_msg);
		left_foot_position_offset_out = vect_msg;
		tf::vector3TFToMsg(fly_off, vect_msg);
		right_foot_position_offset_out = vect_msg;
	}
	else {
		support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_right;
		fly_off = trunk_to_flying_foot_goal.getOrigin() - tf_vec_left;
		tf::vector3TFToMsg(fly_off, vect_msg);
		left_foot_position_offset_out = vect_msg;
		tf::vector3TFToMsg(support_off, vect_msg);
		right_foot_position_offset_out = vect_msg;
	}
	tf::vector3TFToMsg(support_off, vect_msg);
	support_foot_position_offset_out = vect_msg;
	tf::vector3TFToMsg(fly_off, vect_msg);
	fly_foot_position_offset_out = vect_msg;
}

geometry_msgs::Pose KickEngineNodeService::get_trunk_result()
{
	//TODO: Implementation
	geometry_msgs::Pose pose;
	geometry_msgs::Point point;
	point.x = 0;
	point.y = 0;
	point.z = 0;
	pose.position = point;

	return pose;
}

geometry_msgs::Pose KickEngineNodeService::get_pose(Eigen::Vector3d position, Eigen::Vector3d axis)
{
	geometry_msgs::Pose pose;

	tf::pointEigenToMsg(position, pose.position);
	pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(axis[0], axis[1], axis[2]);

	return pose;
}

geometry_msgs::Pose KickEngineNodeService::get_step_pose(Eigen::Vector3d step_position)
{
	geometry_msgs::Pose pose;

	pose.position.x = step_position[0];
	pose.position.y = step_position[1];
	pose.position.z = 0;

	pose.orientation = tf::createQuaternionMsgFromYaw(step_position[2]);

	return pose;
}

Eigen::Vector3d KickEngineNodeService::get_last_footstep()
{
	//TODO: Implementation
	//return _walkEngine.getFootstep().getLast();
	return Eigen::Vector3d();
}

Eigen::Vector3d KickEngineNodeService::get_next_footstep()
{
	//TODO: Implementation
	//return _walkEngine.getFootstep().getNext();
	return Eigen::Vector3d();
}
