#include "KickEngineNodeService.hpp"


std::vector<double> KickEngineNodeService::get_joint_goals(robot_state::RobotStatePtr & goal_state)
{
    //TODO: Implementation
}

bool KickEngineNodeService::convert_goal_coordinate_from_support_foot_to_trunk_based(KickEngine & kick_engine, robot_state::RobotStatePtr & goal_state_out)
{
    //TODO: Implementation
}

void KickEngineNodeService::get_odemetry_data(KickEngine & kick_engine, tf::Vector3 & position_out, geometry_msgs::Quaternion & quaternion_msg_out)
{
    //TODO: Implementation
}
