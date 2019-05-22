#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

void KickEngineNode::kick()
{
    // TODO implementation
}

void KickEngineNode::initialise_ros_subcribtions()
{
    // TODO implementation
}

void KickEngineNode::kick_ball(geometry_msgs::Vector3 const &const ball_position, geometry_msgs::Vector3 const &const target_position)
{
    // TODO implementation

    /*
    KickEngineService kick_engine_service;

    kick_engine_service.set_ball(ball_position.x, ball_position.y, ball_position.z);
    kick_engine_service.set_target(target_position.x, target_position.y, target_position.z);
    kick_engine_service.calc_kick();

    publish_kick(&kick_engine_service);
    */
}

void KickEngineNode::publish_kick()
{
    // TODO implementation
}

void KickEngineNode::robot_state_callback(const humanoid_league_msgs::RobotControlState msg)
{
    // TODO implementation
}

void KickEngineNode::kick_callback()
{
    // TODO implementation
}

int main(int argc, char **argv)
{
}