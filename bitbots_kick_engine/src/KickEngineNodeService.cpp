#include "KickEngineNodeService.hpp"

KickEngineNodeService::KickEngineNodeService()
	: m_kick_engine(), m_debug_service(m_kick_engine)
{
	//TODO: testing
	//TODO: cleanup

	m_debug = nullptr;

	// we have to set some good initial position in the goal state, since we are using a gradient
	// based method. Otherwise, the first step will be not correct
	std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
	std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
	m_kick_engine.set_goal_state(names_vec, pos_vec);

	m_kick_engine.reset_current_state();

	m_bio_ik_solver = bitbots_ik::BioIKSolver(m_kick_engine.get_joint_model_group("All"),
											  m_kick_engine.get_joint_model_grou("LeftLeg"),
											  m_kick_engine.get_joint_model_grou("RightLeg"));
	m_bio_ik_solver.set_use_approximate(true);
}

KickEngineDebugService &KickEngineNodeService::get_debug_service()
{
	return m_debug_service;
}

void KickEngineNodeService::set_robot_state(const humanoid_league_msgs::RobotControlState msg)
{
	//TODO: testing
	//TODO: cleanup

	m_kick_engine.set_robot_state(msg.state);
}

bool KickEngineNodeService::kick(geometry_msgs::Vector3 &ball_position, geometry_msgs::Vector3 &target_position)
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

void KickEngineNodeService::get_goal_feet_joints(std::vector<double> &joint_goals_out, std::vector<std::string> &joint_names_out)
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

	if (m_debug_service.is_debug_on())
	{
		m_debug_service.set_trunk_to_support_foot_goal(trunk_to_support_foot_goal);
		m_debug_service.set_trunk_to_flying_foot_goal(trunk_to_flying_foot_goal);
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

void KickEngineNodeService::get_odemetry_data(tf::Vector3 &position_out, geometry_msgs::Quaternion &quaternion_msg_out)
{
	//TODO: testing
	//TODO: cleanup

	// transformation from support leg to trunk
	auto support_to_trunk = m_kick_engine.get_goal_global_link_transform(get_support_foot_sole()).inverse();
	tf::Transform tf_support_to_trunk;
	tf::transformEigenToTF(support_to_trunk, tf_support_to_trunk);

	// odometry to trunk is transform to support foot * transform from support to trunk
	auto next_step = m_kick_engine.get_next_foot_step();
	double x = next_step[0];
	double y = next_step[1] + m_kick_engine.get_foot_distance() / 2;
	double yaw = next_step[2];

	tf::Transform supportFootTf;
	supportFootTf.setOrigin(tf::Vector3{x, y, 0.0});
	tf::Quaternion supportFootQuat = tf::Quaternion();
	supportFootQuat.setRPY(0, 0, yaw);
	supportFootTf.setRotation(supportFootQuat);
	tf::Transform odom_to_trunk = supportFootTf * tf_support_to_trunk;

	position_out = odom_to_trunk.getOrigin();
	tf::quaternionTFToMsg(odom_to_trunk.getRotation().normalize(), quaternion_msg_out);
}

void KickEngineNodeService::reconfigure_parameter(bitbots_kick_engine::bitbots_quintic_walk_paramsConfig &config, uint32_t level)
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
