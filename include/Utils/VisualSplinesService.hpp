#include <stdint.h>
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "bitbots_splines/Spline.hpp"

class VisualSplinesService
{
public:
    VisualSplinesService(int argc, char** argv);
    ~VisualSplinesService();

    visualization_msgs::Marker get_marker(std::string marker_namespace,
                                            uint8_t marker_id);
    void set_marker_properties(visualization_msgs::Marker & marker,
                                bitbots_splines::Spline * x_spline = NULL,
                                bitbots_splines::Spline * y_spline = NULL,
                                bitbots_splines::Spline * z_spline = NULL);
    void publish_marker(visualization_msgs::Marker & marker);

private:
    uint32_t get_shape();
    void set_marker_scale(visualization_msgs::Marker & marker);
    void set_marker_color(visualization_msgs::Marker & marker);

    uint32_t m_uint_shape;
    ros::Publisher m_ros_marker_publisher;
    double m_d_time;
};
