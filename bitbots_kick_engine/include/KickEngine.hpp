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
	void set_goal_state(std::vector<std::string> vec_joint_names, std::vector<double> vec_position);
    void set_robot_state(const humanoid_league_msgs::RobotControlState msg);
    void kick(geometry_msgs::Vector3 & ball_position, geometry_msgs::Vector3 & target_position);

    std::vector<std::string> get_joint_names() const;
    std::vector<double> get_joint_goals() const;
    geometry_msgs::Twist get_twist() const;
    robot_state::RobotStatePtr get_goal_state() const;
    bool has_new_goals() const;

private:
    geometry_msgs::Vector3 get_kick_start_position();
    void move_left_feet_to_position(geometry_msgs::Vector3 position);
    void move_right_feet_to_position(geometry_msgs::Vector3 position);
    void move_feet_to_position(geometry_msgs::Vector3 position);

    uint8_t m_robot_state;
	robot_model::RobotModelPtr m_kinematic_model;
    robot_state::RobotStatePtr m_goal_state;
    Eigen::Vector3d m_v3d_current_orders;
    bitbots_quintic_walk::WalkingParameter m_parameter;
};

#endif