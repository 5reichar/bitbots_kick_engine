#include "Utils/VisualSplinesService.hpp"

VisualSplinesService::VisualSplinesService(int argc, char** argv, std::string node_name, std::string publisher_topic)
: m_d_time(0.0),
  m_uint_shape(UINT32_MAX)
{
    // init ros
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;

    m_ros_marker_publisher = node_handle.advertise<visualization_msgs::Marker>(publisher_topic, 1);
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

void VisualSplinesService::set_marker(visualization_msgs::Marker & marker, std::string marker_frame_id, std::string marker_namespace, uint8_t marker_id)
{
    //Set the frame ID and timestamp
    marker.header.frame_id = marker_frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = marker_namespace;
    marker.id = marker_id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = get_shape();
}

void VisualSplinesService::set_marker_properties(visualization_msgs::Marker & marker, bitbots_splines::Spline const * const x_spline, bitbots_splines::Spline const * const y_spline, bitbots_splines::Spline const * const z_spline)
{
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    set_marker_position(marker, x_spline, y_spline, z_spline);
    set_marker_orientation(marker);
    set_marker_scale(marker);
    set_marker_color(marker);
    marker.lifetime = ros::Duration();
}

void VisualSplinesService::set_marker_position(visualization_msgs::Marker & marker, bitbots_splines::Spline const * const x_spline, bitbots_splines::Spline const * const y_spline, bitbots_splines::Spline const * const z_spline)
{
    auto x_position = x_spline == NULL ? 0.0 : x_spline->pos(m_d_time);
    auto y_position = y_spline == NULL ? 0.0 : y_spline->pos(m_d_time);
    auto z_position = z_spline == NULL ? 0.0 : z_spline->pos(m_d_time);
    m_d_time += 1;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x_position;
    marker.pose.position.y = y_position;
    marker.pose.position.z = z_position;
}

void VisualSplinesService::set_marker_orientation(visualization_msgs::Marker & marker)
{
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
}

void VisualSplinesService::set_marker_scale(visualization_msgs::Marker & marker)
{
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
}

void VisualSplinesService::set_marker_color(visualization_msgs::Marker & marker)
{
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
}

void VisualSplinesService::publish_marker(visualization_msgs::Marker & marker)
{
    wait_till_someone_subscribed();
    m_ros_marker_publisher.publish(marker);
}

void VisualSplinesService::wait_till_someone_subscribed()
{
    while(m_ros_marker_publisher.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return;
        }

        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
}