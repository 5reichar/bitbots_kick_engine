#ifndef KICKENGINENODE_HPP
#define KICKENGINENODE_HPP

#include <humanoid_league_msgs/RobotControlState.h>

class KickEngineNode
{
public:
    KickEngineNode(/* args */);
    ~KickEngineNode();

    void kick();

private:
    void initialise_ros_subcribtions();
    void kick_ball(geometry_msgs::Vector3 const &const ball_position, geometry_msgs::Vector3 const &const target_position);
    void publish_kick();

    void robot_state_callback(const humanoid_league_msgs::RobotControlState msg);
    void kick_callback();

    humanoid_league_msgs::RobotControlState::state m_robot_state;
};

#endif