#ifndef THROW_ROS_H
#define THROW_ROS_H

#include "ros/ros.h"
#include "engine/throw_engine.h"
#include "parameter/throw_node_parameter.h"

class ThrowRos
{
    // TODO cleanup
public:
    ThrowRos(/* args */);
	~ThrowRos();

    void run_throw(struct3d & ball_position, struct3d & goal_position);
private:

    std::shared_ptr<ThrowNodeParameter> sp_node_parameter_;
    std::shared_ptr<ThrowEngine> sp_throw_engine_;
};

#endif