#ifndef DUBUG_PUBLISHER_H
#define DUBUG_PUBLISHER_H

#include "ros/ros.h"
#include "../../bitbots_spline/include/utils/abstract_visualizer.h"

class DebugPublisher : public bitbots_splines::AbstractVisualizer
{
    // TODO cleanup
public:
    DebugPublisher(ros::NodeHandle & ros_node_handle);

    void publish();
    void publish_markers();

private:
    int32_t marker_id_;

    ros::Publisher ros_publisher_debug_;
    ros::Publisher ros_publisher_debug_engine_;
    ros::Publisher ros_publisher_debug_marker_;
};

#endif