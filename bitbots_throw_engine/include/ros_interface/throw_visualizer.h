#ifndef BITBOTS_THROW_THROW_VISUALIZER_H
#define BITBOTS_THROW_THROW_VISUALIZER_H

#include <../../bitbots_splines_extension/include/abstract/abstract_visualizer.h>
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"
#include "utility/throw_struct.h"
#include <ros/ros.h>
#include "parameter/throw_node_parameter.h"

namespace bitbots_throw{
    class ThrowVisualizer : public bitbots_splines::AbstractVisualizer{
    public:
        struct ThrowVisualizerParams{
            std::string left_hand_frame_;
            std::string left_hand_topic_suffix_;
            std::string left_hand_arrow_topic_suffix_;
            std::string right_hand_frame_;
            std::string right_hand_topic_suffix_;
            std::string right_hand_arrow_topic_suffix_;
            std::string left_foot_frame_;
            std::string left_foot_topic_suffix_;
            std::string left_foot_arrow_topic_suffix_;
            std::string right_foot_frame_;
            std::string right_foot_topic_suffix_;
            std::string right_foot_arrow_topic_suffix_;
        };

        explicit ThrowVisualizer(std::string const & base_topic, ThrowVisualizerParams const & params, std::shared_ptr<ThrowNodeParameter> & node_parameter);

        void update_node_parameter(std::shared_ptr<ThrowNodeParameter> & node_parameter);

        void display_left_hand(std::shared_ptr<bitbots_splines::PoseHandle> & pose);
        void display_right_hand(std::shared_ptr<bitbots_splines::PoseHandle> & pose);
        void display_left_foot(std::shared_ptr<bitbots_splines::PoseHandle> & pose);
        void display_right_foot(std::shared_ptr<bitbots_splines::PoseHandle> & pose);

    private:
        void display_pose(std::shared_ptr<bitbots_splines::PoseHandle> & pose, std::string const & frame, ros::Publisher const & publisher
                          , const Color & color);

        void display_pose_as_arrows(std::shared_ptr<bitbots_splines::PoseHandle> & pose, std::string const & frame, ros::Publisher const & publisher
                                    , const Color & color_1, const Color & color_2, const bool & use_gradient);
        void display_pose_as_arrows(std::shared_ptr<bitbots_splines::PoseHandle> & pose, std::string const & frame, ros::Publisher const & publisher
                                    , const std::vector<double> & pose_times, const Color & color_1, const Color & color_2, const bool & use_gradient);

        visualization_msgs::Marker get_arrow(const geometry_msgs::Pose &pose, const std::string &frame);
        visualization_msgs::Marker get_path(bitbots_splines::PoseHandle & pose, const std::string & frame, const double smoothness);

        ThrowVisualizerParams parameter_;
        std::shared_ptr<ThrowNodeParameter> sp_node_parameter;
        ros::NodeHandle ros_node_handle_;
        ros::Publisher ros_publisher_left_hand_;
        ros::Publisher ros_publisher_left_hand_arrow_;
        ros::Publisher ros_publisher_right_hand_;
        ros::Publisher ros_publisher_right_hand_arrow_;
        ros::Publisher ros_publisher_left_foot_;
        ros::Publisher ros_publisher_left_foot_arrow_;
        ros::Publisher ros_publisher_right_foot_;
        ros::Publisher ros_publisher_right_foot_arrow_;
    };
}
#endif //BITBOTS_THROW_THROW_VISUALIZER_H
