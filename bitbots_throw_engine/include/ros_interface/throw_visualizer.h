#ifndef BITBOTS_THROW_THROW_VISUALIZER_H
#define BITBOTS_THROW_THROW_VISUALIZER_H

#include <../../bitbots_splines_extension/include/abstract/abstract_visualizer.h>
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"
#include "utility/throw_struct.h"
#include <ros/ros.h>

namespace bitbots_throw{
    class ThrowVisualizer : public bitbots_splines::AbstractVisualizer{
    public:
        struct ThrowVisualizerParams{
            double smoothness_;
            std::string left_hand_frame;
            std::string right_hand_frame;
            std::string left_foot_frame;
            std::string right_foot_frame;
        };

        explicit ThrowVisualizer(std::string const & base_topic, ThrowVisualizerParams const & params);

        void update_smoothness(double const & smoothness);

        void display_left_hand(std::shared_ptr<bitbots_splines::PoseHandle> & pose);
        void display_right_hand(std::shared_ptr<bitbots_splines::PoseHandle> & pose);
        void display_left_foot(std::shared_ptr<bitbots_splines::PoseHandle> & pose);
        void display_right_foot(std::shared_ptr<bitbots_splines::PoseHandle> & pose);

    private:
        void display_pose(std::shared_ptr<bitbots_splines::PoseHandle> & pose, std::string const & frame, ros::Publisher const & publisher
                          , double const & color_r, double const & color_g, double const & color_b);

        ThrowVisualizerParams parameter_;
        ros::NodeHandle ros_node_handle_;
        ros::Publisher ros_publisher_left_hand_;
        ros::Publisher ros_publisher_right_hand_;
        ros::Publisher ros_publisher_left_foot_;
        ros::Publisher ros_publisher_right_foot_;
    };
}
#endif //BITBOTS_THROW_THROW_VISUALIZER_H
