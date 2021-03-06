#ifndef BITBOTS_THROW_DUBUG_PUBLISHER_H
#define BITBOTS_THROW_DUBUG_PUBLISHER_H

#include "ros/ros.h"
#include <moveit_msgs/RobotState.h>
#include "bitbots_splines/abstract_visualizer.h"
#include "bitbots_splines/abstract_ik.h"
#include "parameter/throw_engine_parameter.h"

#include "utility/throw_utilities.h"

namespace bitbots_throw{
    class DebugPublisher : public bitbots_splines::AbstractVisualizer{
    public:
        explicit DebugPublisher(ros::NodeHandle & ros_node_handle
                               ,std::string const & topic
                               ,std::string const & topic_marker);

        void print_throw_points(std::string point_data, std::shared_ptr<ThrowDebugParameter> const & sp_parameter);
        void publish_ik_debug(ThrowResponse const & response, int8_t const & percentage_done, int8_t const & movement_stage);

    private:
        void publish_arrow_marker(std::string const & name_space
                                 ,std::string const & frame
                                 ,geometry_msgs::Pose const & pose
                                 ,float const & r
                                 ,float const & g
                                 ,float const & b
                                 ,float const & a);

        int32_t marker_id_;

        ros::Publisher ros_publisher_debug_;
        ros::Publisher ros_publisher_debug_marker_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_DUBUG_PUBLISHER_H