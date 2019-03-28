#include <VisualSplinesService.hpp>
#include <ros/ros.h>

VisualSplinesService::VisualSplinesService(int argc, char** argv)
{
    m_uint_shape = UINT32_MAX;
    m_ros_marker_publisher = node_handle.advertise<visualization_msgs::Maker>("spline_marker", 1);

    // init ros
    ros::init(argc, argv, "spline_visual");
    ros::NodeHandle node_handle;
}

VisualSplinesService::~VisualSplinesService()
{
}

uint32_t VisualSplinesService::get_shape()
{
    switch (m_uint_shape)
    {
        case visualization_msgs::Marker::CUBE:
            m_uint_shape = visualization_msgs::Marker::SPHERE;
            break;
        case visualization_msgs::Marker::SPHERE:
            m_uint_shape = visualization_msgs::Marker::ARROW;
            break;
        case visualization_msgs::Marker::ARROW:
            m_uint_shape = visualization_msgs::Marker::CYLINDER;
            break;
        case visualization_msgs::Marker::CYLINDER:
        default:
            m_uint_shape = visualization_msgs::Marker::CUBE;
            break;
    }

    return m_uint_shape;
}

visualization_msgs::Marker VisualSplinesService::get_marker(string marker_namespace, uint8_t marker_id)
{
    visualization_msgs::Marker marker;

    //Set the frame ID and timestamp
    marker.header.frame_id = "spline_visual_frame";
    marker.header.stamp = ros::time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = marker_namespace;
    marker.id = marker_id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = get_shape();
}

void VisualSplinesService::set_marker_properties(visualization_msgs::Marker & marker)
{
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    set_marker_scale();
    set_marker_color();
    marker.lifetime = ros::Duration();
}

void VisualSplinesService::set_marker_scale()
{
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
}

void VisualSplinesService::set_marker_color()
{
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
}

void VisualSplinesService::publish_marker(visualization_msgs::Marker & marker)
{
    while(m_ros_marker_publisher.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return 0;
        }

        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    m_ros_marker_publisher.publish(marker);
}