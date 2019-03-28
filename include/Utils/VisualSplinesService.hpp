#include <stdint.h>
#include <visualization_msg/Marker.h>
#include <Spline.hpp>

class VisualSplinesService
{
public:
    VisualSplinesService(int argc, char** argv);
    ~VisualSplinesService();

    visualization_msgs::Marker get_marker(string marker_namespace, uint8_t marker_id);
    void set_marker_properties(visualization_msgs::Marker & marker);
    void publish_marker(visualization_msgs::Marker & marker);

private:
    uint32_t get_shape();
    void set_marker_scale();
    void set_marker_color();

    uint32_t m_uint_shape;
    ros::Publisher m_ros_marker_publisher;
};
