#ifndef KICKENGINENODESERVICE_HPP
#define KICKENGINENODESERVICE_HPP

#include "KickEngine.hpp"
#include "bitbots_ik/BioIKSolver.hpp"
#include <geometry_msgs/Quaternion.h>

class KickEngineNodeService
{
public:

    std::vector<double> get_joint_goals(robot_state::RobotStatePtr & goal_state);
    bool convert_goal_coordinate_from_support_foot_to_trunk_based(KickEngine & kick_engine, robot_state::RobotStatePtr & goal_state_out);
    void get_odemetry_data(KickEngine & kick_engine, tf::Vector3 & position_out, geometry_msgs::Quaternion & quaternion_msg_out);

private:

    bitbots_ik::BioIKSolver m_bio_ik_solver;
};

#endif