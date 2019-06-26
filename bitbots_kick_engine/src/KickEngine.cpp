#include "KickEngine.hpp"

KickEngine::KickEngine()
{
    // TODO: implementation
}

void KickEngine::set_param()
{
    // TODO: implementation
}

void KickEngine::set_robot_state(const humanoid_league_msgs::RobotControlState msg)
{
    // TODO: implementation
}

void KickEngine::kick(geometry_msgs::Vector3 & ball_position, geometry_msgs::Vector3 & target_position)
{
    // TODO: implementation
}

std::vector<std::string> KickEngine::get_joint_names() const
{
    // TODO: implementation
}

std::vector<double> KickEngine::get_joint_goals() const
{
    // TODO: implementation
}

geometry_msgs::Twist KickEngine::get_twist() const
{
    // TODO: implementation

    geometry_msgs::Twist twist;
    twist.linear.x = m_v3d_current_orders[0] * m_parameter.freq * 2;
    twist.linear.y = m_v3d_current_orders[1] * m_parameter.freq * 2;
    twist.angular.z = m_v3d_current_orders[2] * m_parameter.freq * 2;

    return twist;
}


robot_state::RobotStatePtr KickEngine::get_goal_state() const
{
    // TODO: implementation
}

bool KickEngine::has_new_goals() const
{
    // TODO: implementation
}

geometry_msgs::Vector3 KickEngine::get_kick_start_position()
{
    // TODO: implementation
}

void KickEngine::move_left_feet_to_position(geometry_msgs::Vector3 position)
{
    // TODO: implementation
}

void KickEngine::move_right_feet_to_position(geometry_msgs::Vector3 position)
{
    // TODO: implementation
}

void KickEngine::move_feet_to_position(geometry_msgs::Vector3 position)
{
    // TODO: implementation
}
