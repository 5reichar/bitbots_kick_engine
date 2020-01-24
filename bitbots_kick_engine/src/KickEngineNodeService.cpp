#include "KickEngineNodeService.hpp"
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

KickEngineNodeService::KickEngineNodeService(bool simulation)
	: m_sp_debug_service(new KickEngineDebugService(m_sp_kick_engine))
{
	//TODO: testing
	//TODO: cleanup

	m_b_simulation_active = simulation;

	// we have to set some good initial position in the goal state, since we are using a gradient
	// based method. Otherwise, the first step will be not correct
	std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
	std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
	m_sp_kick_engine->setGoalState(names_vec, pos_vec);

	m_sp_kick_engine->resetCurrentState();

	m_bio_ik_solver = bitbots_ik::BioIKSolver(m_sp_kick_engine->getJointModelGroup("All"),
											  m_sp_kick_engine->getJointModelGroup("LeftLeg"),
											  m_sp_kick_engine->getJointModelGroup("RightLeg"));
	m_bio_ik_solver.set_use_approximate(true);
}

bool KickEngineNodeService::convertGoalCoordinateFromSupportFootToTrunkBased()
{
	//TODO: testing
	//TODO: cleanup

	robot_state::RobotStatePtr goal_state;

	// change goals from support foot based coordinate system to trunk based coordinate system
	auto trunk_to_support_foot_goal = getSupportFootTransformation(m_sp_kick_engine->getTrunkPosition(), m_sp_kick_engine->getTrunkAxis()).inverse();
	auto trunk_to_flying_foot_goal = trunk_to_support_foot_goal * getSupportFootTransformation(m_sp_kick_engine->getFlyFootPosition(), m_sp_kick_engine->getFlyFootAxis());

	// call ik solver
	bool success = m_bio_ik_solver.solve(trunk_to_support_foot_goal, trunk_to_flying_foot_goal, isLeftFootSupport(), goal_state);

	m_sp_kick_engine->setGoalState(goal_state);

	if (m_sp_debug_service->isDebugOn())
	{
		m_sp_debug_service->setTrunkToSupportFootGoal(trunk_to_support_foot_goal);
		m_sp_debug_service->setTrunkToFlyingFootGoal(trunk_to_flying_foot_goal);
	}

	return success;
}

bool KickEngineNodeService::kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position)
{
	//TODO: testing
	//TODO: cleanup

	bool success = false;

	if (m_sp_kick_engine->update(calculateTimeDelta()))
	{
		m_sp_kick_engine->kick(ball_position, target_position);
		success = true;
	}

	return success;
}

double KickEngineNodeService::calculateTimeDelta()
{
	//TODO: testing
	//TODO: cleanup

	// compute time delta depended if we are currently in simulation or reality
	double dt;

	if (!m_b_simulation_active)
	{
		std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();
		// only take real time difference if walking was not stopped before
		// using c++ time since it is more performant than ros time. We only need a local difference, so it doesnt matter as long as we are not simulating
		auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - m_time_point_last_update);
		dt = time_diff_ms.count() / 1000.0;
		if (dt == 0)
		{
			ROS_WARN("dt was 0");
			dt = 0.001;
		}
		m_time_point_last_update = current_time;
	}
	else
	{
		ROS_WARN_ONCE("Simulation active, using ROS time");
		// use ros time for simulation
		double current_ros_time = ros::Time::now().toSec();
		dt = current_ros_time - m_d_ros_time_last_update;
		m_d_ros_time_last_update = current_ros_time;
	}

	// time is wrong when we run it for the first time
	if (m_b_first_run)
	{
		m_b_first_run = false;
		dt = 0.0001;
	}
	return dt;
}

KickPreparationPosition KickEngineNodeService::makePosition(struct3d position, double robot_sector_min, double robot_sector_max, double ball_sector_min, double ball_sector_max)
{
	AngleRequirements robot_sector = {robot_sector_min, robot_sector_max};
	AngleRequirements ball_sector = {ball_sector_min, ball_sector_max};
	KickPreparationPosition prepare_position = {position, robot_sector, ball_sector};
	return prepare_position;
}

void KickEngineNodeService::reconfigureKickPreparationPositions(bitbots_kick_engine::kick_preparation_positionsConfig& config, uint32_t level)
{
	//TODO: testing
	//TODO: cleanup

	auto kick_parameter = m_sp_kick_engine->getKickParameter();

	// Positions are put in the array, so now the index matches the enum int of the position in the config file
	struct3d positions[9];
	positions[0] = {config.default_position_x, config.default_position_y, config.default_position_z};
	positions[1] = {config.straight_front_position_x, config.straight_front_position_y, config.straight_front_position_z};
	positions[2] = {config.straight_back_position_x, config.straight_back_position_y, config.straight_back_position_z};
	positions[3] = {config.straight_left_position_x, config.straight_left_position_y, config.straight_left_position_z};
	positions[4] = {config.straight_right_position_x, config.straight_right_position_y, config.straight_right_position_z};
	positions[5] = {config.side_front_left_position_x, config.side_front_left_position_y, config.side_front_left_position_z};
	positions[6] = {config.side_front_right_position_x, config.side_front_right_position_y, config.side_front_right_position_z};
	positions[7] = {config.side_back_left_position_x, config.side_back_left_position_y, config.side_back_left_position_z};
	positions[8] = {config.side_back_right_position_x, config.side_back_right_position_y, config.side_back_right_position_z};

	kick_parameter->default_kick_preparation_position = positions[0];

	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs1_ball_sector_1_use_kp_position],
																		 config.robot_sector_1_angle_min,
																		 config.robot_sector_1_angle_max,
																		 config.rs1_ball_sector_1_angle_min,
																		 config.rs1_ball_sector_1_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs1_ball_sector_2_use_kp_position],
																		 config.robot_sector_1_angle_min,
																		 config.robot_sector_1_angle_max,
																		 config.rs1_ball_sector_2_angle_min,
																		 config.rs1_ball_sector_2_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs1_ball_sector_3_use_kp_position],
																		 config.robot_sector_1_angle_min,
																		 config.robot_sector_1_angle_max,
																		 config.rs1_ball_sector_3_angle_min,
																		 config.rs1_ball_sector_3_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs1_ball_sector_4_use_kp_position],
																		 config.robot_sector_1_angle_min,
																		 config.robot_sector_1_angle_max,
																		 config.rs1_ball_sector_4_angle_min,
																		 config.rs1_ball_sector_4_angle_max));

	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs2_ball_sector_1_use_kp_position],
																		 config.robot_sector_2_angle_min,
																		 config.robot_sector_2_angle_max,
																		 config.rs2_ball_sector_1_angle_min,
																		 config.rs2_ball_sector_1_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs2_ball_sector_2_use_kp_position],
																		 config.robot_sector_2_angle_min,
																		 config.robot_sector_2_angle_max,
																		 config.rs2_ball_sector_2_angle_min,
																		 config.rs2_ball_sector_2_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs2_ball_sector_3_use_kp_position],
																		 config.robot_sector_2_angle_min,
																		 config.robot_sector_2_angle_max,
																		 config.rs2_ball_sector_3_angle_min,
																		 config.rs2_ball_sector_3_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs2_ball_sector_4_use_kp_position],
																		 config.robot_sector_2_angle_min,
																		 config.robot_sector_2_angle_max,
																		 config.rs2_ball_sector_4_angle_min,
																		 config.rs2_ball_sector_4_angle_max));

	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs3_ball_sector_1_use_kp_position],
																		 config.robot_sector_3_angle_min,
																		 config.robot_sector_3_angle_max,
																		 config.rs3_ball_sector_1_angle_min,
																		 config.rs3_ball_sector_1_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs3_ball_sector_2_use_kp_position],
																		 config.robot_sector_3_angle_min,
																		 config.robot_sector_3_angle_max,
																		 config.rs3_ball_sector_2_angle_min,
																		 config.rs3_ball_sector_2_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs3_ball_sector_3_use_kp_position],
																		 config.robot_sector_3_angle_min,
																		 config.robot_sector_3_angle_max,
																		 config.rs3_ball_sector_3_angle_min,
																		 config.rs3_ball_sector_3_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs3_ball_sector_4_use_kp_position],
																		 config.robot_sector_3_angle_min,
																		 config.robot_sector_3_angle_max,
																		 config.rs3_ball_sector_4_angle_min,
																		 config.rs3_ball_sector_4_angle_max));

	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs4_ball_sector_1_use_kp_position],
																		 config.robot_sector_4_angle_min,
																		 config.robot_sector_4_angle_max,
																		 config.rs4_ball_sector_1_angle_min,
																		 config.rs4_ball_sector_1_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs4_ball_sector_2_use_kp_position],
																		 config.robot_sector_4_angle_min,
																		 config.robot_sector_4_angle_max,
																		 config.rs4_ball_sector_2_angle_min,
																		 config.rs4_ball_sector_2_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs4_ball_sector_3_use_kp_position],
																		 config.robot_sector_4_angle_min,
																		 config.robot_sector_4_angle_max,
																		 config.rs4_ball_sector_3_angle_min,
																		 config.rs4_ball_sector_3_angle_max));
	kick_parameter->v_kick_preparation_positions.push_back(makePosition(positions[config.rs4_ball_sector_4_use_kp_position],
																		 config.robot_sector_4_angle_min,
																		 config.robot_sector_4_angle_max,
																		 config.rs4_ball_sector_4_angle_min,
																		 config.rs4_ball_sector_4_angle_max));

}

void KickEngineNodeService::reconfigureEngineParameter(bitbots_kick_engine::kick_engine_paramsConfig& config, uint32_t level)
{
	//TODO: testing
	//TODO: cleanup

	auto engine_parameter = m_sp_kick_engine->getEngineParameter();

	engine_parameter->freq = config.freq;
	engine_parameter->footDistance = config.footDistance;
	engine_parameter->engineFrequency = config.engineFreq;

	m_bio_ik_solver.set_bioIK_timeout(config.bioIKTime);
}

KickType KickEngineNodeService::makeKickType(KickTypeId id, bool active,  double robot_req_angle_min, double robot_req_angle_max, double ball_req_angle_min, double ball_req_angle_max)
{
	AngleRequirements robot_req_angle = {robot_req_angle_min, robot_req_angle_max};
	AngleRequirements ball_req_angle = {ball_req_angle_min, ball_req_angle_max};
	KickType type = {id, active, robot_req_angle, ball_req_angle};
	return type;
}

void KickEngineNodeService::reconfigureKickParameter(bitbots_kick_engine::kick_paramsConfig& config, uint32_t level)
{
	//TODO: testing
	//TODO: cleanup

	auto kick_parameter = m_sp_kick_engine->getKickParameter();

	kick_parameter->default_kick_id =(KickTypeId) config.default_kick_use_kick_enum;

	kick_parameter->v_kick_types.push_back(makeKickType((KickTypeId) config.beziercurve_kick_enum,
										   				  config.beziercurve_kick_active,
										   				  config.beziercurve_kick_angle_requirement_min_robot_ball,
										   				  config.beziercurve_kick_angle_requirement_max_robot_ball,
										   				  config.beziercurve_kick_angle_requirement_min_ball_goal,
										   				  config.beziercurve_kick_angle_requirement_max_ball_goal));

	kick_parameter->v_kick_types.push_back(makeKickType((KickTypeId) config.linear_spline_kick_enum,
										   				  config.linear_spline_kick_active,
										   				  config.linear_spline_kick_angle_requirement_min_robot_ball,
										   				  config.linear_spline_kick_angle_requirement_max_robot_ball,
										   				  config.linear_spline_kick_angle_requirement_min_ball_goal,
										   				  config.linear_spline_kick_angle_requirement_max_ball_goal));

	kick_parameter->v_kick_types.push_back(makeKickType((KickTypeId) config.cubic_spline_kick_enum,
										   				  config.cubic_spline_kick_active,
										   				  config.cubic_spline_kick_angle_requirement_min_robot_ball,
										   				  config.cubic_spline_kick_angle_requirement_max_robot_ball,
										   				  config.cubic_spline_kick_angle_requirement_min_ball_goal,
										   				  config.cubic_spline_kick_angle_requirement_max_ball_goal));

	kick_parameter->v_kick_types.push_back(makeKickType((KickTypeId) config.smooth_spline_kick_enum,
										   				  config.smooth_spline_kick_active,
										   				  config.smooth_spline_kick_angle_requirement_min_robot_ball,
										   				  config.smooth_spline_kick_angle_requirement_max_robot_ball,
										   				  config.smooth_spline_kick_angle_requirement_min_ball_goal,
										   				  config.smooth_spline_kick_angle_requirement_max_ball_goal));
}

geometry_msgs::Vector3 KickEngineNodeService::createVector3(float x, float y, float z)
{
	// TODO testing
	// TODO cleanup

	geometry_msgs::Vector3 scale;

	scale.x = x;
	scale.y = y;
	scale.z = z;

	return scale;
}

std_msgs::ColorRGBA KickEngineNodeService::createColorRGBA(float red, float green, float blue, float alpha)
{
	// TODO testing
	// TODO cleanup

	std_msgs::ColorRGBA color;

	color.r = red;
	color.g = green;
	color.b = blue;
	color.a = alpha;

	return color;
}

bool KickEngineNodeService::isLeftFootSupport()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->isLeftFootSupport();
}

bool KickEngineNodeService::areBoothFeetSupport()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->areBoothFeetSupport();
}

void KickEngineNodeService::setRobotState(const humanoid_league_msgs::RobotControlState msg)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_kick_engine->setRobotState(msg.state);
}

std::string KickEngineNodeService::getSupportFootSole() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->getSupportFootSole();
}

geometry_msgs::Twist KickEngineNodeService::getTwist() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->getTwist();
}

double KickEngineNodeService::getEngineFrequence() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->getEngineParameter()->engineFrequency;
}

geometry_msgs::Pose KickEngineNodeService::getTrunkResult()
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

std_msgs::Char KickEngineNodeService::getSupportFootState()
{
	//TODO: testing
	//TODO: cleanup

	std_msgs::Char support_foot_state;

	if (areBoothFeetSupport())
	{
		support_foot_state.data = 'd';
	}
	else if (isLeftFootSupport())
	{
		support_foot_state.data = 'l';
	}
	else
	{
		support_foot_state.data = 'r';
	}

	return support_foot_state;
}

std::shared_ptr<KickEngineDebugService> KickEngineNodeService::getDebugService()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_debug_service;
}

void KickEngineNodeService::getOdemetryData(tf::Vector3& position_out, geometry_msgs::Quaternion& quaternion_msg_out)
{
	//TODO: testing
	//TODO: cleanup

	// transformation from support leg to trunk
	auto support_to_trunk = m_sp_kick_engine->getGoalGlobalLinkTransform(m_sp_kick_engine->getSupportFootSole()).inverse();
	tf::Transform tf_support_to_trunk;
	tf::transformEigenToTF(support_to_trunk, tf_support_to_trunk);

	// odometry to trunk is transform to support foot * transform from support to trunk
	auto next_step = m_sp_kick_engine->getNextFootStep();
	double x = next_step[0];
	double y = next_step[1] + m_sp_kick_engine->getEngineParameter()->footDistance / 2;
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

void KickEngineNodeService::getGoalFeetJoints(std::vector<double> &joint_goals_out, std::vector<std::string> &joint_names_out)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_kick_engine->getGoalJointGroup("Legs", joint_goals_out, joint_names_out);
}

geometry_msgs::Pose KickEngineNodeService::getLastFootstepPose()
{
	//TODO: testing
	//TODO: cleanup

	return getPoseFromStep(m_sp_kick_engine->getLastFootStep());
}

geometry_msgs::Pose KickEngineNodeService::getNextFootstepPose()
{
	//TODO: testing
	//TODO: cleanup

	return getPoseFromStep(m_sp_kick_engine->getNextFootStep());
}

tf::Transform KickEngineNodeService::getSupportFootTransformation(Eigen::Vector3d position, Eigen::Vector3d axis)
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

geometry_msgs::Pose KickEngineNodeService::getPoseFromStep(Eigen::Vector3d step_position)
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
