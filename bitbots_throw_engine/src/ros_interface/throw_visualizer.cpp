#include "ros_interface/throw_visualizer.h"
#include "utility/throw_struct.h"

namespace bitbots_throw{
    ThrowVisualizer::ThrowVisualizer(const std::string & base_topic, ThrowVisualizerParams const & params
                                     , std::shared_ptr<ThrowNodeParameter> & node_parameter)
            : sp_node_parameter(node_parameter), parameter_(params){
        ros_publisher_left_hand_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + params.left_hand_topic_suffix_, 100);
        ros_publisher_right_hand_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + params.right_hand_topic_suffix_, 100);
        ros_publisher_left_foot_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + params.left_foot_topic_suffix_, 100);
        ros_publisher_right_foot_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + params.right_foot_topic_suffix_, 100);

        ros_publisher_left_hand_arrow_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + params.left_hand_arrow_topic_suffix_, 100);
        ros_publisher_right_hand_arrow_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + params.right_hand_arrow_topic_suffix_, 100);
        ros_publisher_left_foot_arrow_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + params.left_foot_arrow_topic_suffix_, 100);
        ros_publisher_right_foot_arrow_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + params.right_foot_arrow_topic_suffix_, 100);
    }

    void ThrowVisualizer::update_node_parameter(std::shared_ptr<ThrowNodeParameter> & node_parameter){
        sp_node_parameter = node_parameter;
    }

    void ThrowVisualizer::display_left_hand(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
        if(sp_node_parameter->visualize_limbs_){
            display_pose(pose, parameter_.left_hand_frame_, ros_publisher_left_hand_
                         , {1, 1, 0});
        }

        if(sp_node_parameter->visualize_left_arm_){
            display_pose_as_arrows(pose, parameter_.left_hand_frame_, ros_publisher_left_hand_arrow_
                                   , {1, 1, 1}, {0, 0, 0}
                                   , sp_node_parameter->visualize_arrows_use_gradient_);
        }
    }

    void ThrowVisualizer::display_right_hand(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
        if(sp_node_parameter->visualize_limbs_){
            display_pose(pose, parameter_.right_hand_frame_, ros_publisher_right_hand_
                         , {1, 0, 1});
        }


        if(sp_node_parameter->visualize_right_arm_){
            display_pose_as_arrows(pose, parameter_.right_hand_frame_, ros_publisher_right_hand_arrow_
                                   , {1, 1, 1}, {0, 0, 0}
                                   , sp_node_parameter->visualize_arrows_use_gradient_);
        }
    }

    void ThrowVisualizer::display_left_foot(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
        if(sp_node_parameter->visualize_limbs_){
            display_pose(pose, parameter_.left_foot_frame_, ros_publisher_left_foot_
                         , {0, 1, 0});
        }

        if(sp_node_parameter->visualize_left_foot_){
            display_pose_as_arrows(pose, parameter_.left_foot_frame_, ros_publisher_left_foot_arrow_
                                   , {1, 1, 1}, {0, 0, 0}
                                   , sp_node_parameter->visualize_arrows_use_gradient_);
        }
    }

    void ThrowVisualizer::display_right_foot(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
        if(sp_node_parameter->visualize_limbs_){
            display_pose(pose, parameter_.right_foot_frame_, ros_publisher_right_foot_
                         , {0, 0, 1});
        }

        if(sp_node_parameter->visualize_right_foot_){
            display_pose_as_arrows(pose, parameter_.right_foot_frame_, ros_publisher_right_foot_arrow_
                                   , {1, 1, 1}, {0, 0, 0}
                                   , sp_node_parameter->visualize_arrows_use_gradient_);
        }
    }

    void ThrowVisualizer::display_pose(std::shared_ptr<bitbots_splines::PoseHandle> & pose
                                       , const std::string & frame
                                       , ros::Publisher const & publisher
                                       , const Color & color){
        auto path = getPath(*pose, frame, sp_node_parameter->visualization_smoothness_);

        path.color.r = color.red_;
        path.color.g = color.green_;
        path.color.b = color.blue_;
        path.ns = "throw_debug";

        publisher.publish(path);
    }

    void ThrowVisualizer::display_pose_as_arrows(std::shared_ptr<bitbots_splines::PoseHandle> & pose
                                                 , const std::string & frame, const ros::Publisher & publisher
                                                 , const Color & color_1, const Color & color_2
                                                 , const bool & use_gradient){
        uint32_t index = 0;
        Color color_helper = {use_gradient ? color_1.red_ : 1.0, color_1.green_, color_1.blue_};
        Color color_step = color_2 - color_1;
        color_step /= sp_node_parameter->visualization_smoothness_;
        double pose_min = pose->x()->min();
        double pose_max = pose->x()->max();
        double time_step = (pose_max - pose_min) / sp_node_parameter->visualization_smoothness_;

        for(auto i = pose_min; i <= pose_max; i += time_step){
            auto arrow = get_arrow(pose->get_geometry_msg_pose(i), frame);
            arrow.ns = "throw_debug";
            arrow.id = index;
            ++index;

            if(use_gradient){
                arrow.color.r = color_helper.red_;
                arrow.color.g = color_helper.green_;
                arrow.color.b = color_helper.blue_;

                color_helper += color_step;
            }else{
                arrow.color.r = color_helper.red_ == 1.0 ? color_1.red_ : color_2.red_;
                arrow.color.g = color_helper.red_ == 1.0 ? color_1.green_ : color_2.green_;
                arrow.color.b = color_helper.red_ == 1.0 ? color_1.blue_ : color_2.blue_;

                color_helper.red_ *= -1;
            }

            publisher.publish(arrow);
        }
    }

    visualization_msgs::Marker ThrowVisualizer::get_arrow(const geometry_msgs::Pose & pose, const std::string & frame){
        visualization_msgs::Marker marker;

        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.lifetime = ros::Duration(1000);
        marker.frame_locked = false;
        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time::now();
        marker.pose = pose;
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        marker.color.a = 1;

        return marker;
    }
}
