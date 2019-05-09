#include <stdint.h>
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "Utils/VisualSplinesMaterial.hpp"

enum Color
{
    red,
    green,
    blue,
    yellow,
    white
};

class VisualSplinesService
{
public:
    VisualSplinesService(int argc, char **argv, std::string node_name, std::string publisher_topic);
    ~VisualSplinesService();

    void set_marker_frame(std::string marker_frame_id);

    void set_marker(visualization_msgs::Marker &marker,
                    std::string marker_namespace,
                    uint8_t marker_id,
                    uint32_t const &marker_shape);

    void set_marker_position(visualization_msgs::Marker &marker_lines,
                             uint32_t const number_of_points,
                             VisualSplinesMaterial *vs_material,
                             double const step = 1.0);

    void draw_points(visualization_msgs::Marker &marker_points,
                     VisualSplinesMaterial *vs_material);

    void set_marker_color(visualization_msgs::Marker &marker, Color const color);
    void set_marker_scale(visualization_msgs::Marker &marker, float const x = 1.0, float const y = 1.0, float const z = 1.0);
    void publish_marker(visualization_msgs::Marker &marker);

private:
    void wait_till_someone_subscribed();
    void add_point_to_marker(visualization_msgs::Marker &marker, double x, double y, double z);

    double m_d_time;
    std::string m_str_marker_frame_id;
    ros::Publisher m_ros_marker_publisher;
};
