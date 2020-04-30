#include "ros_interface/publisher/debug_publisher.h"
#include <visualization_msgs/Marker.h>
#include <bitbots_throw_engine/throw_debug.h>
#include <bitbots_throw_engine/throw_engine_debug.h>

DebugPublisher::DebugPublisher(ros::NodeHandle & ros_node_handle)
{
    ros_publisher_debug_ = ros_node_handle.advertise<bitbots_throw_engine::throw_debug>("throw_debug", 1);
    ros_publisher_debug_engine_ = ros_node_handle.advertise<bitbots_throw_engine::throw_engine_debug>("throw_engine_debug", 1);
	ros_publisher_debug_marker_ = ros_node_handle.advertise<visualization_msgs::Marker>("throw_debug_marker", 1);
}

void DebugPublisher::publish()
{
    //TODO: implement
}

void DebugPublisher::publish_markers()
{
    //TODO: implement
}