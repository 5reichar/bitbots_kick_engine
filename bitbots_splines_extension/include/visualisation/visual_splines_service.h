#ifndef BITBOTS_SPLINES_EXTENSION_VISUAL_SPLINES_SERVICE_H
#define BITBOTS_SPLINES_EXTENSION_VISUAL_SPLINES_SERVICE_H

#include <stdint.h>
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "visualisation/visual_splines_material.h"

class VisualSplinesService
{
public:
    VisualSplinesService(int argc, char **argv, std::string node_name, std::string publisher_topic);
    ~VisualSplinesService();

    void set_marker_frame(std::string marker_frame_id);

    void visualize_curve(VisualSplinesMaterial *vs_material);
    void visualize_points(VisualSplinesMaterial *vs_material,
                          std::string const str_namespace,
                          Color const color,
                          double const scale = 0.3,
                          uint32_t id = 0);

    void set_marker_properties(visualization_msgs::Marker &marker,
                               std::string marker_namespace,
                               uint8_t marker_id,
                               uint32_t const &marker_shape);

    void calc_and_add_points_from_curve(visualization_msgs::Marker &marker_lines,
                                        VisualSplinesMaterial *vs_material,
                                        double const sampling_rate = 10.0);

    void add_points_from_curve(visualization_msgs::Marker &marker_points,
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

#endif