#ifndef KICKENGINE_HPP
#define KICKENGINE_HPP

#include <humanoid_league_msgs/RobotControlState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

class KickEngine
{
public:
    enum class KickType : uint16_t
    {
        standard = 0
    };

    KickEngine();

    void set_param();
	virtual void set_goal_state(robot_state::RobotStatePtr& goal_state);
	virtual void set_goal_state(std::vector<std::string> vec_joint_names, std::vector<double> vec_position);
    void set_robot_state(const humanoid_league_msgs::RobotControlState msg);
    void kick(geometry_msgs::Vector3 & ball_position, geometry_msgs::Vector3 & target_position);
	void reset_current_state();

    std::vector<std::string> get_joint_names() const;
    std::vector<double> get_joint_goals() const;
    geometry_msgs::Twist get_twist() const;
    robot_state::RobotStatePtr get_goal_state() const;
    bool has_new_goals() const;
	bool is_left_foot_support() const;
	bool are_booth_feet_support() const;
	robot_state::JointModelGroup& get_joint_model_group(std::string name);
	Eigen::Vector3d get_fly_foot_position() const;
	Eigen::Vector3d get_fly_foot_axis() const;
	Eigen::Vector3d get_trunk_position() const;
	Eigen::Vector3d get_trunk_axis() const;
	void get_goal_joint_group(std::string joint_group_name, std::vector<double>& joint_goals_out, std::vector<std::string>& joint_names_out);
	Eigen::Isometry3d get_goal_global_link_transform(std::string link_name);
	Eigen::Isometry3d get_current_global_link_transform(std::string link_name);
	Eigen::Vector3d get_next_foot_step();
	Eigen::Vector3d get_last_foot_step();
	double get_foot_distance() const;

	double get_phase_time() const;
	double get_trajectory_time() const;
	std_msgs::String get_state() const;

private:
    geometry_msgs::Vector3 get_kick_start_position();
    void move_left_feet_to_position(geometry_msgs::Vector3 position);
    void move_right_feet_to_position(geometry_msgs::Vector3 position);
    void move_feet_to_position(geometry_msgs::Vector3 position);

    uint8_t m_robot_state;
	robot_model::RobotModelPtr m_kinematic_model;
    robot_state::RobotStatePtr m_goal_state;
	robot_state::RobotStatePtr m_current_state;
    Eigen::Vector3d m_v3d_current_orders;
    bitbots_quintic_walk::WalkingParameter m_parameter;
};

#endif