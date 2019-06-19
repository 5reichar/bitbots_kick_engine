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
