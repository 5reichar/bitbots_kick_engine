#include "Utils/VisualSplinesService.hpp"

VisualSplinesService::VisualSplinesService(int argc, char **argv, std::string node_name, std::string publisher_topic)
{
    // init ros
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;

    this->m_ros_marker_publisher = node_handle.advertise<visualization_msgs::Marker>(publisher_topic, 10);
}

VisualSplinesService::~VisualSplinesService()
{
}

void VisualSplinesService::set_marker_frame(std::string marker_frame_id)
{
    this->m_str_marker_frame_id = marker_frame_id;
}

void VisualSplinesService::set_marker(visualization_msgs::Marker &marker,
                                      std::string marker_namespace,
                                      uint8_t marker_id,
                                      uint32_t const &marker_shape)
{
    //Set the frame ID and timestamp
    marker.header.frame_id = this->m_str_marker_frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = marker_namespace;
    marker.id = marker_id;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = marker_shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;
}

void VisualSplinesService::set_marker_position(visualization_msgs::Marker &marker_points,
                                               visualization_msgs::Marker &marker_lines,
                                               uint32_t const number_of_points,
                                               VisualSplinesMaterial *vs_material,
                                               double const step,
                                               bool const debug)
{
    for (uint32_t i = 0; i < number_of_points; ++i)
    {
        double time = i * step;
        geometry_msgs::Point point;
        point.x = vs_material->get_position_from_x(time);
        point.y = vs_material->get_position_from_y(time);
        point.z = vs_material->get_position_from_z(time);

        marker_points.points.push_back(point);
        marker_lines.points.push_back(point);
    }
}

void VisualSplinesService::set_marker_scale(visualization_msgs::Marker &marker, float const x, float const y, float const z)
{
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
}

void VisualSplinesService::set_marker_color(visualization_msgs::Marker &marker, Color const color)
{
    switch (color)
    {
    case red:
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        break;

    case green:
    default:
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        break;

    case blue:
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        break;

    case yellow:
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        break;
    }

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.a = 1.0;
}

void VisualSplinesService::publish_marker(visualization_msgs::Marker &marker)
{
    wait_till_someone_subscribed();
    this->m_ros_marker_publisher.publish(marker);
}

void VisualSplinesService::wait_till_someone_subscribed()
{
    while (this->m_ros_marker_publisher.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return;
        }

        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
}