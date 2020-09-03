#include "ros_interface/throw_visualizer.h"

bitbots_throw::ThrowVisualizer::ThrowVisualizer(const std::string & base_topic, ThrowVisualizerParams const & params){
    parameter_ = params;
    ros_publisher_left_hand_ = ros_node_handle_.advertise<visualization_msgs::Marker>(base_topic + "_left_hand_marker", 100);
    ros_publisher_right_hand_ = ros_node_handle_.advertise<visualization_msgs::Marker>(base_topic + "_right_hand_marker", 100);
    ros_publisher_left_foot_ = ros_node_handle_.advertise<visualization_msgs::Marker>(base_topic + "_left_foot_marker", 100);
    ros_publisher_right_foot_ = ros_node_handle_.advertise<visualization_msgs::Marker>(base_topic + "_right_foot_marker", 100);
}

void bitbots_throw::ThrowVisualizer::update_smoothness(const double & smoothness){
    parameter_.smoothness_ = smoothness;
}

void bitbots_throw::ThrowVisualizer::display_left_hand(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
    display_pose(pose, parameter_.left_hand_frame, ros_publisher_left_hand_, 1, 1, 0);
}

void bitbots_throw::ThrowVisualizer::display_right_hand(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
    display_pose(pose, parameter_.right_hand_frame, ros_publisher_right_hand_, 1, 0, 1);
}

void bitbots_throw::ThrowVisualizer::display_left_foot(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
    display_pose(pose, parameter_.left_foot_frame, ros_publisher_left_foot_, 0, 1, 0);
}

void bitbots_throw::ThrowVisualizer::display_right_foot(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
    display_pose(pose, parameter_.right_foot_frame, ros_publisher_right_foot_, 0, 0, 1);
}

void bitbots_throw::ThrowVisualizer::display_pose(std::shared_ptr<bitbots_splines::PoseHandle> & pose
                                                  , const std::string & frame
                                                  , ros::Publisher const & publisher
                                                  , const double & color_r
                                                  , const double & color_g
                                                  , const double & color_b){
    auto path = getPath(*pose, frame, parameter_.smoothness_);

    path.color.r = color_r;
    path.color.g = color_g;
    path.color.b = color_b;
    path.ns = "throw_debug";

    publisher.publish(path);
}
