#ifndef KICKENGINENODE_HPP
#define KICKENGINENODE_HPP

#include "KickEngine.hpp"

class KickEngineNode
{
public:
    KickEngineNode(/* args */);
    ~KickEngineNode();

    void kick();

private:
    void initialise_ros_subcribtions();
    void initialise_ros_publisher();
    void kick_ball(geometry_msgs::Vector3 const &const ball_position, geometry_msgs::Vector3 const &const target_position);
    void publish_kick() const;
    void publish_odemetry() const;
    void publish_controler_commands(std::vector<std::string> joint_names, std::vector<double> positions) const;

    void robot_state_callback(const humanoid_league_msgs::RobotControlState msg);
    void kick_callback();

    KickEngine m_kick_enginge;
    uint16_t m_uint_odometry_publish_factor;
};

#endif