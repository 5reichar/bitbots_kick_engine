#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include "bitbots_kick_engine/KickEngineService.hpp"

void publish_kick(KickEngineService *kick_engine_service)
{
    //TODO implement publishing of kick
}

void kick_ball(geometry_msgs::Vector3 const &const ball_position, geometry_msgs::Vector3 const &const target_position)
{
    KickEngineService kick_engine_service;

    kick_engine_service.set_ball(ball_position.x, ball_position.y, ball_position.z);
    kick_engine_service.set_target(target_position.x, target_position.y, target_position.z);
    kick_engine_service.calc_kick();

    publish_kick(&kick_engine_service);
}

int main(int argc, char **argv)
{
}