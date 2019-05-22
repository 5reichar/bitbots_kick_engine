#include "visualisation/VisualSplinesService.hpp"

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

void VisualSplinesService::visualize_points(VisualSplinesMaterial *vs_material,
                                            std::string const str_namespace,
                                            Color const color,
                                            double const scale,
                                            uint32_t id)
{

    visualization_msgs::Marker vs_marker_points;

    set_marker_properties(vs_marker_points, str_namespace, id, visualization_msgs::Marker::POINTS);
    set_marker_scale(vs_marker_points, scale, scale, scale);
    set_marker_color(vs_marker_points, color);

    add_points_from_curve(vs_marker_points, vs_material);

    publish_marker(vs_marker_points);
}

void VisualSplinesService::visualize_curve(VisualSplinesMaterial *vs_material)
{
    visualization_msgs::Marker vs_marker_lines;

    set_marker_properties(vs_marker_lines, vs_material->get_namspace(), vs_material->get_id(), visualization_msgs::Marker::LINE_STRIP);
    set_marker_scale(vs_marker_lines, vs_material->get_scale_x(), vs_material->get_scale_y(), vs_material->get_scale_z());
    set_marker_color(vs_marker_lines, vs_material->get_color());

    calc_and_add_points_from_curve(vs_marker_lines, vs_material);

    publish_marker(vs_marker_lines);
}

void VisualSplinesService::set_marker_properties(visualization_msgs::Marker &marker,
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

    case white:
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        break;
    }

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.a = 1.0;
}

void VisualSplinesService::calc_and_add_points_from_curve(visualization_msgs::Marker &marker_lines,
                                                          VisualSplinesMaterial *vs_material,
                                                          double const sampling_rate)
{
    auto intervall_end = vs_material->get_points()[0].back().time;
    auto step = 1 / sampling_rate;

    for (double time = 0; time < intervall_end; time += step)
    {
        add_point_to_marker(marker_lines,
                            vs_material->get_position_from_x(time),
                            vs_material->get_position_from_y(time),
                            vs_material->get_position_from_z(time));
    }
}

void VisualSplinesService::add_points_from_curve(visualization_msgs::Marker &marker_points,
                                                 VisualSplinesMaterial *vs_material)
{
    auto vec_points = vs_material->get_points();
    auto it_points_x = vec_points[0].begin();
    auto it_points_y = vec_points[1].begin();
    auto it_points_z = vec_points[2].begin();

    while (it_points_x != vec_points[0].end() && it_points_y != vec_points[1].end() && it_points_z != vec_points[2].end())
    {
        add_point_to_marker(marker_points, (*it_points_x).position, (*it_points_y).position, (*it_points_z).position);

        ++it_points_x;
        ++it_points_y;
        ++it_points_z;
    }
}

void VisualSplinesService::add_point_to_marker(visualization_msgs::Marker &marker, double x, double y, double z)
{
    geometry_msgs::Point point;

    point.x = x;
    point.y = y;
    point.z = z;

    marker.points.push_back(point);
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