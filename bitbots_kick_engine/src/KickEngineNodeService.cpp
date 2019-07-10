#include "KickEngineNodeService.hpp"


KickEngineNodeService::KickEngineNodeService()
{
	//TODO: testing
	//TODO: cleanup

	m_debug = nullptr;

	// we have to set some good initial position in the goal state, since we are using a gradient
	// based method. Otherwise, the first step will be not correct
	std::vector<std::string> names_vec = { "LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch" };
	std::vector<double> pos_vec = { 0.7, -1.0, -0.4, -0.7, 1.0, 0.4 };
	m_kick_engine.set_goal_state(names_vec, pos_vec);

	m_kick_engine.reset_current_state();

	m_bio_ik_solver = bitbots_ik::BioIKSolver(m_kick_engine.get_joint_model_group("All"),
		m_kick_engine.get_joint_model_grou("LeftLeg"),
		m_kick_engine.get_joint_model_grou("RightLeg"));
	m_bio_ik_solver.set_use_approximate(true);
}

KickEngineNodeService::~KickEngineNodeService()
{
	if (m_debug)
	{
		delete m_debug;
	}
}

void KickEngineNodeService::set_debug(bool debug)
{
	if (debug && !m_debug)
	{
		m_debug = new KENSdebug();
	}
	else if (!debug && m_debug)
	{
		delete m_debug;
		m_debug = nullptr;
	}
}

void KickEngineNodeService::set_robot_state(const humanoid_league_msgs::RobotControlState msg)
{
	//TODO: testing
	//TODO: cleanup

	m_kick_engine.set_robot_state(msg.state);
}

bool KickEngineNodeService::kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position)
{
	//TODO: testing
	//TODO: cleanup

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
	//TODO: testing
	//TODO: cleanup

	return m_kick_engine.is_left_foot_support();
}

bool KickEngineNodeService::are_booth_feet_support()
{
	//TODO: testing
	//TODO: cleanup

	return m_kick_engine.are_booth_feet_support();
}

geometry_msgs::Pose KickEngineNodeService::get_last_footstep_pose()
{
	//TODO: testing
	//TODO: cleanup

	return get_pose_from_step(get_last_footstep());
}

geometry_msgs::Pose KickEngineNodeService::get_next_footstep_pose()
{
	//TODO: testing
	//TODO: cleanup

	return get_pose_from_step(get_next_footstep());
}

geometry_msgs::Pose KickEngineNodeService::get_engine_fly_foot_goal_pose()
{
	//TODO: testing
	//TODO: cleanup

	return get_pose(m_kick_engine.get_fly_foot_position(), m_kick_engine.get_fly_foot_axis());
}

geometry_msgs::Pose KickEngineNodeService::get_engine_trunk_goal_pose()
{
	//TODO: testing
	//TODO: cleanup

	return get_pose(m_kick_engine.get_trunk_position(), m_kick_engine.get_trunk_axis());
}

std_msgs::Char KickEngineNodeService::get_support_foot_sole()
{
	//TODO: testing
	//TODO: cleanup

	return is_left_foot_support() ? "l_sole" : "r_sole";
}

void KickEngineNodeService::get_goal_feet_joints(std::vector<double>& joint_goals_out, std::vector<std::string>& joint_names_out)
{
	//TODO: testing
	//TODO: cleanup

	m_kick_engine.get_goal_joint_group("Legs", joint_goals_out, joint_names_out);
}

std_msgs::Char KickEngineNodeService::get_support_foot_state()
{
	//TODO: testing
	//TODO: cleanup

	std_msgs::Char support_foot_state;

	if (are_booth_feet_support())
	{
		support_state.data = 'd';
	}
	else if (is_left_foot_support())
	{
		support_state.data = 'l';
	}
	else
	{
		support_state.data = 'r';
	}

	return support_foot_state
}

bool KickEngineNodeService::convert_goal_coordinate_from_support_foot_to_trunk_based()
{
	//TODO: testing
	//TODO: cleanup

	 // read the cartesian positions and orientations for trunk and fly foot
	m_kick_engine.computeCartesianPosition(_trunkPos, _trunkAxis, _footPos, _footAxis, _isLeftSupport);
	robot_state::RobotStatePtr goal_state;

	// change goals from support foot based coordinate system to trunk based coordinate system
	auto trunk_to_support_foot_goal = get_support_foot_transformation(m_kick_engine.get_trunk_position(), m_kick_engine.get_trunk_axis()).inverse();
	auto trunk_to_flying_foot_goal = trunk_to_support_foot_goal * get_support_foot_transformation(m_kick_engine.get_fly_foot_position(), m_kick_engine.get_fly_foot_axis());

	// call ik solver
	bool success = m_bio_ik_solver.solve(trunk_to_support_foot_goal, trunk_to_flying_foot_goal,
		is_left_foot_support(), goal_state);

	m_kick_engine.set_goal_state(goal_state);

	if (m_debug)
	{
		m_debug->m_tf_trunk_to_support_foot_goal = trunk_to_support_foot_goal;
		m_debug->m_tf_trunk_to_flying_foot_goal = trunk_to_flying_foot_goal;
	}

	return success;
}

tf::Transform KickEngineNodeService::get_support_foot_transformation(Eigen::Vector3d position, Eigen::Vector3d axis)
{
	//TODO: testing
	//TODO: cleanup

	tf::Vector3 tf_vec;
	tf::vectorEigenToTF(position, tf_vec);

	tf::Quaternion tf_quat = tf::Quaternion();
	tf_quat.setRPY(axis[0], axis[1], axis[2]);
	tf_quat.normalize();

	tf::Transform support_foot_transformation(tf_quat, tf_vec);

	return support_foot_transformation;
}

void KickEngineNodeService::get_odemetry_data(tf::Vector3 & position_out, geometry_msgs::Quaternion & quaternion_msg_out)
{
	//TODO: testing
	//TODO: cleanup

	// transformation from support leg to trunk
	auto support_to_trunk = m_kick_engine.get_goal_global_link_transform(get_support_foot_sole()).inverse();
	tf::Transform tf_support_to_trunk;
	tf::transformEigenToTF(support_to_trunk, tf_support_to_trunk);

	// odometry to trunk is transform to support foot * transform from support to trunk
	auto next_step = get_next_footstep();
	double x = next_step[0];
	double y = next_step[1] + m_kick_engine.get_foot_distance() / 2;
	double yaw = next_step[2];

	tf::Transform supportFootTf;
	supportFootTf.setOrigin(tf::Vector3{ x, y, 0.0 });
	tf::Quaternion supportFootQuat = tf::Quaternion();
	supportFootQuat.setRPY(0, 0, yaw);
	supportFootTf.setRotation(supportFootQuat);
	tf::Transform odom_to_trunk = supportFootTf * tf_support_to_trunk;

	position_out = odom_to_trunk.getOrigin();
	tf::quaternionTFToMsg(odom_to_trunk.getRotation().normalize(), quaternion_msg_out);
}

double KickEngineNodeService::get_phase_time()
{
	//TODO: testing
	//TODO: cleanup

	return m_kick_engine.get_phase_time();
}

double KickEngineNodeService::get_trajectory_time()
{
	//TODO: testing
	//TODO: cleanup

	return m_kick_engine.get_trajectory_time();
}

std_msgs::String KickEngineNodeService::get_engine_state()
{
	//TODO: testing
	//TODO: cleanup

	return m_kick_engine.get_state();
}

bool KickEngineNodeService::get_feet_goals(geometry_msgs::Pose& left_foot_goal_out, geometry_msgs::Pose& right_foot_goal_out, geometry_msgs::Pose& fly_foot_goal_out, geometry_msgs::Pose& support_foot_goal_out)
{
	//TODO: testing
	//TODO: cleanup

	bool success = false;

	if (m_debug)
	{
		geometry_msgs::Pose pose_support_foot_goal;
		tf::pointTFToMsg(m_debug->m_tf_trunk_to_support_foot_goal.getOrigin(), pose_support_foot_goal.position);
		tf::quaternionTFToMsg(m_debug->m_tf_trunk_to_support_foot_goal.getRotation(), pose_support_foot_goal.orientation);
		support_foot_goal_out = pose_support_foot_goal;

		geometry_msgs::Pose pose_fly_foot_goal;
		tf::pointTFToMsg(m_debug->m_tf_trunk_to_flying_foot_goal.getOrigin(), pose_fly_foot_goal.position);
		tf::quaternionTFToMsg(m_debug->m_tf_trunk_to_flying_foot_goal.getRotation(), pose_fly_foot_goal.orientation);
		fly_foot_goal_out = pose_fly_foot_goal;

		if (is_left_support) {
			left_foot_goal_out = pose_support_foot_goal;
			right_foot_goal_out = pose_fly_foot_goal;
		}
		else {
			left_foot_goal_out = pose_fly_foot_goal;
			right_foot_goal_out = pose_support_foot_goal;
		}

		success = true;
	}

	return success;
}

void KickEngineNodeService::get_feet_ik_results(geometry_msgs::Pose& left_foot_ik_result_out, geometry_msgs::Pose& right_foot_ik_result_out, geometry_msgs::Pose& fly_foot_ik_result_out, geometry_msgs::Pose& support_foot_ik_result_out)
{
	//TODO: testing
	//TODO: cleanup

	get_feet_position(&m_kick_engine.get_goal_global_link_transform, left_foot_ik_result_out, right_foot_ik_result_out, fly_foot_ik_result_out, support_foot_ik_result_out);
}

void KickEngineNodeService::get_feet_position(geometry_msgs::Pose& left_foot_position_out, geometry_msgs::Pose& right_foot_position_out, geometry_msgs::Pose& fly_foot_position_out, geometry_msgs::Pose& support_foot_position_out)
{
	//TODO: testing
	//TODO: cleanup

	get_feet_position(&m_kick_engine.get_current_global_link_transform _current_state, left_foot_position_out, right_foot_position_out, fly_foot_position_out, support_foot_position_out);
}

void KickEngineNodeService::get_feet_position(Eigen::Isometry3d(KickEngine::* get_global_link_transform) (std::string link_name), geometry_msgs::Pose& left_foot_out, geometry_msgs::Pose& right_foot_out, geometry_msgs::Pose& fly_foot_out, geometry_msgs::Pose& support_foot_out)
{
	//TODO: testing
	//TODO: cleanup

	geometry_msgs::Pose left_foot;
	tf::poseEigenToMsg(m_kick_engine->*get_global_link_transform("l_sole"), left_foot);
	left_foot_out = left_foot;

	geometry_msgs::Pose right_foot;
	tf::poseEigenToMsg(m_kick_engine->*get_global_link_transform("r_sole"), right_foot);
	right_foot_out = right_foot;

	if (is_left_foot_support())
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

void KickEngineNodeService::get_feet_ik_offset(geometry_msgs::Vector3& left_foot_ik_offset_out, geometry_msgs::Vector3& right_foot_ik_offset_out, geometry_msgs::Vector3& fly_foot_ik_offset_out, geometry_msgs::Vector3& support_foot_ik_offset_out)
{
	//TODO: testing
	//TODO: cleanup

	get_feet_offset(&m_kick_engine.get_goal_global_link_transform, left_foot_ik_offset_out, right_foot_ik_offset_out, fly_foot_ik_offset_out, support_foot_ik_offset_out);
}

void KickEngineNodeService::get_feet_position_offset(geometry_msgs::Vector3& left_foot_position_offset_out, geometry_msgs::Vector3& right_foot_position_offset_out, geometry_msgs::Vector3& fly_foot_position_offset_out, geometry_msgs::Vector3& support_foot_position_offset_out)
{
	//TODO: testing
	//TODO: cleanup

	get_feet_offset(&m_kick_engine.get_current_global_link_transform, left_foot_position_offset_out, right_foot_position_offset_out, fly_foot_position_offset_out, support_foot_position_offset_out);
}

void KickEngineNodeService::get_feet_offset(Eigen::Isometry3d(KickEngine::* get_global_link_transform) (std::string link_name), geometry_msgs::Vector3& left_foot_offset_out, geometry_msgs::Vector3& right_foot_offset_out, geometry_msgs::Vector3& fly_foot_offset_out, geometry_msgs::Vector3& support_foot_offset_out)
{
	//TODO: testing
	//TODO: cleanup

	tf::Vector3 left_foot;
	tf::vectorEigenToTF(m_kick_engine->*get_global_link_transform("l_sole").translation(), left_foot);

	tf::Vector3 right_foot;
	tf::vectorEigenToTF(m_kick_engine->*get_global_link_transform("r_sole").translation(), right_foot);

	tf::Vector3 fly_off;
	tf::Vector3 support_off;
	if (is_left_foot_support())
	{
		support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_left;
		fly_off = trunk_to_flying_foot_goal.getOrigin() - tf_vec_right;
		tf::vector3TFToMsg(support_off, vect_msg);
		left_foot_offset_out = vect_msg;
		tf::vector3TFToMsg(fly_off, vect_msg);
		right_foot_offset_out = vect_msg;
	}
	else
	{
		support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_right;
		fly_off = trunk_to_flying_foot_goal.getOrigin() - tf_vec_left;
		tf::vector3TFToMsg(fly_off, vect_msg);
		left_foot_offset_out = vect_msg;
		tf::vector3TFToMsg(support_off, vect_msg);
		right_foot_offset_out = vect_msg;
	}

	tf::vector3TFToMsg(support_off, vect_msg);
	support_foot_offset_out = vect_msg;
	tf::vector3TFToMsg(fly_off, vect_msg);
	fly_foot_offset_out = vect_msg;
}

void KickEngineNodeService::reconfigure_parameter(bitbots_kick_engine::bitbots_quintic_walk_paramsConfig& config, uint32_t level)
{
	//TODO: implemention
	//TODO: testing
	//TODO: cleanup

	kick_engine_parameter parameter;

	parameter.freq = config.freq;
	parameter.doubleSupportRatio = config.doubleSupportRatio;
	parameter.footDistance = config.footDistance;
	parameter.footRise = config.footRise;
	parameter.footZPause = config.footZPause;
	parameter.footPutDownZOffset = config.footPutDownZOffset;
	parameter.footPutDownPhase = config.footPutDownPhase;
	parameter.footApexPhase = config.footApexPhase;
	parameter.footOvershootRatio = config.footOvershootRatio;
	parameter.footOvershootPhase = config.footOvershootPhase;
	parameter.trunkHeight = config.trunkHeight;
	parameter.trunkPitch = config.trunkPitch;
	parameter.trunkPhase = config.trunkPhase;
	parameter.trunkXOffset = config.trunkXOffset;
	parameter.trunkYOffset = config.trunkYOffset;
	parameter.trunkSwing = config.trunkSwing;
	parameter.trunkPause = config.trunkPause;
	parameter.trunkXOffsetPCoefForward = config.trunkXOffsetPCoefForward;
	parameter.trunkXOffsetPCoefTurn = config.trunkXOffsetPCoefTurn;
	parameter.trunkPitchPCoefForward = config.trunkPitchPCoefForward;
	parameter.trunkPitchPCoefTurn = config.trunkPitchPCoefTurn;
	parameter.kickLength = config.kickLength;
	parameter.kickPhase = config.kickPhase;
	parameter.footPutDownRollOffset = config.footPutDownRollOffset;
	parameter.kickVel = config.kickVel;

	parameter.engineFrequency = config.engineFreq;

	parameter.max_step[0] = config.maxStepX;
	parameter.max_step[1] = config.maxStepY;
	parameter.max_step[2] = config.maxStepZ;
	parameter.max_step_xy = config.maxStepXY;

	parameter.imuActive = config.imuActive;
	parameter.imu_pitch_threshold = config.imuPitchThreshold;
	parameter.imu_roll_threshold = config.imuRollThreshold;

	parameter.phaseResetActive = config.phaseResetActive;
	parameter.groundMinPressure = config.groundMinPressure;
	parameter.copStopActive = config.copStopActive;
	parameter.ioPressureThreshold = config.ioPressureThreshold;
	parameter.fbPressureThreshold = config.fbPressureThreshold;
	parameter.pauseDuration = config.pauseDuration;

	m_kick_engine.set_parameter(parameter);
	m_bio_ik_solver.set_bioIK_timeout(config.bioIKTime);
}

double KickEngineNodeService::get_engine_frequence() const
{
	return m_kick_engine.get_engine_frequence();
}

bitbots_quintic_walk::WalkingDebug KickEngineNodeService::create_debug_message()
{
	bitbots_quintic_walk::WalkingDebug msg;

	msg.is_left_support = is_left_foot_support();
	msg.is_double_support = are_booth_feet_support();
	msg.header.stamp = ros::Time::now();

	// times
	msg.phase_time = get_engine_phase();
	msg.traj_time = get_engine_trajectory_time();

	msg.engine_state.data = get_engine_state();

	// engine output
	msg.engine_fly_goal = get_engine_fly_foot_goal_pose();
	msg.engine_trunk_goal = get_engine_trunk_goal_pose();

	// goals
	get_feet_goals(msg.left_foot_goal, msg.right_foot_goal, msg.fly_foot_goal, msg.support_foot_goal);

	// IK results
	get_feet_ik_results(msg.left_foot_ik_result, msg.right_foot_ik_result, msg.fly_foot_ik_result, support_foot_ik_result);

	// IK offsets
	get_feet_ik_offset(msg.left_foot_ik_offset, msg.right_foot_ik_offset, msg.fly_foot_ik_offset, msg.support_foot_ik_offset);

	// actual positions
	get_feet_position(msg.left_foot_position, msg.right_foot_position, msg.fly_foot_position, msg.support_foot_position);

	// actual offsets
	get_feet_position_offset(msg.left_foot_actual_offset, msg.right_foot_actual_offset, msg.fly_foot_actual_offset, msg.support_foot_actual_offset);

	return msg;
}

geometry_msgs::Pose KickEngineNodeService::get_trunk_result()
{
	//TODO: testing
	//TODO: cleanup

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
	//TODO: testing
	//TODO: cleanup

	geometry_msgs::Pose pose;

	tf::pointEigenToMsg(position, pose.position);
	pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(axis[0], axis[1], axis[2]);

	return pose;
}

geometry_msgs::Pose KickEngineNodeService::get_pose_from_step(Eigen::Vector3d step_position)
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

Eigen::Vector3d KickEngineNodeService::get_last_footstep()
{
	//TODO: testing
	//TODO: cleanup

	return m_kick_engine.get_last_foot_step();
}

Eigen::Vector3d KickEngineNodeService::get_next_footstep()
{
	//TODO: testing
	//TODO: cleanup

	return m_kick_engine.get_next_foot_step();
}
