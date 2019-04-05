#include <stdint.h>
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "bitbots_splines/Spline.hpp"

class VisualSplinesService
{
public:
    VisualSplinesService(int argc, char** argv, std::string node_name, std::string publisher_topic);
    ~VisualSplinesService();

    void set_marker(visualization_msgs::Marker & marker,
                        std::string marker_frame_id,
                        std::string marker_namespace,
                        uint8_t marker_id);
    void set_marker_properties(visualization_msgs::Marker & marker,
                                bitbots_splines::Spline const * const x_spline = NULL,
                                bitbots_splines::Spline const * const y_spline = NULL,
                                bitbots_splines::Spline const * const z_spline = NULL);
    void publish_marker(visualization_msgs::Marker & marker);

private:
    void set_marker_position(visualization_msgs::Marker & marker,
                                bitbots_splines::Spline const * const x_spline = NULL,
                                bitbots_splines::Spline const * const y_spline = NULL,
                                bitbots_splines::Spline const * const z_spline = NULL);
    void set_marker_orientation(visualization_msgs::Marker & marker);
    void set_marker_scale(visualization_msgs::Marker & marker);
    void set_marker_color(visualization_msgs::Marker & marker);
    uint32_t get_shape();
    void wait_till_someone_subscribed();

    uint32_t m_uint_shape;
    ros::Publisher m_ros_marker_publisher;
    double m_d_time;
};
