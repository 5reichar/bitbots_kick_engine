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
	m_sp_kick_engine->set_goal_state(names_vec, pos_vec);

	m_sp_kick_engine->reset_current_state();

	m_bio_ik_solver = bitbots_ik::BioIKSolver(m_sp_kick_engine->get_joint_model_group("All"),
											  m_sp_kick_engine->get_joint_model_group("LeftLeg"),
											  m_sp_kick_engine->get_joint_model_group("RightLeg"));
	m_bio_ik_solver.set_use_approximate(true);
}

bool KickEngineNodeService::convert_goal_coordinate_from_support_foot_to_trunk_based()
{
	//TODO: testing
	//TODO: cleanup

	robot_state::RobotStatePtr goal_state;

	// change goals from support foot based coordinate system to trunk based coordinate system
	auto trunk_to_support_foot_goal = get_support_foot_transformation(m_sp_kick_engine->get_trunk_position(), m_sp_kick_engine->get_trunk_axis()).inverse();
	auto trunk_to_flying_foot_goal = trunk_to_support_foot_goal * get_support_foot_transformation(m_sp_kick_engine->get_fly_foot_position(), m_sp_kick_engine->get_fly_foot_axis());

	// call ik solver
	bool success = m_bio_ik_solver.solve(trunk_to_support_foot_goal, trunk_to_flying_foot_goal, is_left_foot_support(), goal_state);

	m_sp_kick_engine->set_goal_state(goal_state);

	if (m_sp_debug_service->is_debug_on())
	{
		m_sp_debug_service->set_trunk_to_support_foot_goal(trunk_to_support_foot_goal);
		m_sp_debug_service->set_trunk_to_flying_foot_goal(trunk_to_flying_foot_goal);
	}

	return success;
}

bool KickEngineNodeService::kick(geometry_msgs::Vector3& ball_position, geometry_msgs::Vector3& target_position)
{
	//TODO: testing
	//TODO: cleanup

	bool success = false;

	if (m_sp_kick_engine->update(calculate_time_delta()))
	{
		m_sp_kick_engine->kick(ball_position, target_position);
		success = true;
	}

	return success;
}

double KickEngineNodeService::calculate_time_delta()
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

void KickEngineNodeService::reconfigure_engine_parameter(bitbots_kick_engine::kick_engine_paramsConfig& config, uint32_t level)
{
	//TODO: testing
	//TODO: cleanup

	auto engine_parameter = m_sp_kick_engine->get_engine_parameter();

	engine_parameter->freq = config.freq;
	engine_parameter->footDistance = config.footDistance;
	engine_parameter->engineFrequency = config.engineFreq;

	m_bio_ik_solver.set_bioIK_timeout(config.bioIKTime);
}

void KickEngineNodeService::reconfigure_kick_parameter(bitbots_kick_engine::kick_paramsConfig& config, uint32_t level)
{
	//TODO: implementation
	//TODO: testing
	//TODO: cleanup

	auto kick_parameter = m_sp_kick_engine->get_kick_parameter();
}

geometry_msgs::Vector3 KickEngineNodeService::create_vector_3(float x, float y, float z)
{
	// TODO testing
	// TODO cleanup

	geometry_msgs::Vector3 scale;

	scale.x = x;
	scale.y = y;
	scale.z = z;

	return scale;
}

std_msgs::ColorRGBA KickEngineNodeService::create_color_rgba(float red, float green, float blue, float alpha)
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

bool KickEngineNodeService::is_left_foot_support()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->is_left_foot_support();
}

bool KickEngineNodeService::are_booth_feet_support()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->are_booth_feet_support();
}

void KickEngineNodeService::set_robot_state(const humanoid_league_msgs::RobotControlState msg)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_kick_engine->set_robot_state(msg.state);
}

std::string KickEngineNodeService::get_support_foot_sole() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->get_support_foot_sole();
}

geometry_msgs::Twist KickEngineNodeService::get_twist() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->get_twist();
}

double KickEngineNodeService::get_engine_frequence() const
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_kick_engine->get_engine_parameter()->engineFrequency;
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

std_msgs::Char KickEngineNodeService::get_support_foot_state()
{
	//TODO: testing
	//TODO: cleanup

	std_msgs::Char support_foot_state;

	if (are_booth_feet_support())
	{
		support_foot_state.data = 'd';
	}
	else if (is_left_foot_support())
	{
		support_foot_state.data = 'l';
	}
	else
	{
		support_foot_state.data = 'r';
	}

	return support_foot_state;
}

std::shared_ptr<KickEngineDebugService> KickEngineNodeService::get_debug_service()
{
	//TODO: testing
	//TODO: cleanup

	return m_sp_debug_service;
}

void KickEngineNodeService::get_odemetry_data(tf::Vector3& position_out, geometry_msgs::Quaternion& quaternion_msg_out)
{
	//TODO: testing
	//TODO: cleanup

	// transformation from support leg to trunk
	auto support_to_trunk = m_sp_kick_engine->get_goal_global_link_transform(m_sp_kick_engine->get_support_foot_sole()).inverse();
	tf::Transform tf_support_to_trunk;
	tf::transformEigenToTF(support_to_trunk, tf_support_to_trunk);

	// odometry to trunk is transform to support foot * transform from support to trunk
	auto next_step = m_sp_kick_engine->get_next_foot_step();
	double x = next_step[0];
	double y = next_step[1] + m_sp_kick_engine->get_engine_parameter()->footDistance / 2;
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

void KickEngineNodeService::get_goal_feet_joints(std::vector<double> &joint_goals_out, std::vector<std::string> &joint_names_out)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_kick_engine->get_goal_joint_group("Legs", joint_goals_out, joint_names_out);
}

geometry_msgs::Pose KickEngineNodeService::get_last_footstep_pose()
{
	//TODO: testing
	//TODO: cleanup

	return get_pose_from_step(m_sp_kick_engine->get_last_foot_step());
}

geometry_msgs::Pose KickEngineNodeService::get_next_footstep_pose()
{
	//TODO: testing
	//TODO: cleanup

	return get_pose_from_step(m_sp_kick_engine->get_next_foot_step());
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
