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
    void initialise_ros_publisher();
    void kick_ball(geometry_msgs::Vector3 const &const ball_position, geometry_msgs::Vector3 const &const target_position);
    void publish_kick() const;
    void publish_odemetry() const;

    void robot_state_callback(const humanoid_league_msgs::RobotControlState msg);
    void kick_callback();
    void velocity_cammand_callback(const geometry_msgs::Twist msg);

    humanoid_league_msgs::RobotControlState::state m_robot_state;
    uint16_t m_uint_odometry_publish_factor;
};

#endif