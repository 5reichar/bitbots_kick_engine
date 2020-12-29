#include "ros_interface/throw_visualizer.h"
#include "utility/throw_struct.h"
#include "ros_interface/ros_joint_and_topic_names.h"

namespace bitbots_throw{
    ThrowVisualizer::ThrowVisualizer(const std::string & base_topic, ThrowVisualizerParams const & params
                                     , std::shared_ptr<ThrowDebugParameter> & sp_parameter)
            : sp_parameter(sp_parameter), parameter_(params){
        ros_publisher_left_hand_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + RosJointAndTopicNames::get_topic_suffix_left_hand() , 100);
        ros_publisher_right_hand_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + RosJointAndTopicNames::get_topic_suffix_right_hand(), 100);
        ros_publisher_left_foot_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + RosJointAndTopicNames::get_topic_suffix_left_foot(), 100);
        ros_publisher_right_foot_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + RosJointAndTopicNames::get_topic_suffix_right_foot(), 100);

        ros_publisher_left_hand_arrow_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + RosJointAndTopicNames::get_topic_suffix_left_hand_arrow(), 100);
        ros_publisher_right_hand_arrow_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + RosJointAndTopicNames::get_topic_suffix_right_hand_arrow(), 100);
        ros_publisher_left_foot_arrow_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + RosJointAndTopicNames::get_topic_suffix_left_foot_arrow(), 100);
        ros_publisher_right_foot_arrow_ = ros_node_handle_.advertise<visualization_msgs::Marker>(
                base_topic + RosJointAndTopicNames::get_topic_suffix_right_foot_arrow(), 100);
    }

    void ThrowVisualizer::set_parameter(std::shared_ptr<ThrowDebugParameter> & sp_parameter){
        sp_parameter = sp_parameter;
    }

    void ThrowVisualizer::display_left_hand(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
        if(sp_parameter->visualize_movement_){
            display_pose(pose, parameter_.left_hand_frame_, ros_publisher_left_hand_
                         , {1, 1, 0});
        }

        if(sp_parameter->visualize_left_arm_arrows_){
            display_pose_as_arrows(pose, parameter_.left_hand_frame_, ros_publisher_left_hand_arrow_
                                   , {1, 1, 1}, {0, 0, 0}
                                   , sp_parameter->visualize_arrows_use_gradient_);
        }
    }

    void ThrowVisualizer::display_right_hand(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
        if(sp_parameter->visualize_movement_){
            display_pose(pose, parameter_.right_hand_frame_, ros_publisher_right_hand_
                         , {1, 0, 1});
        }

        if(sp_parameter->visualize_right_arm_arrows_){
            display_pose_as_arrows(pose, parameter_.right_hand_frame_, ros_publisher_right_hand_arrow_
                                   , {1, 1, 1}, {0, 0, 0}
                                   , sp_parameter->visualize_arrows_use_gradient_);
        }
    }

    void ThrowVisualizer::display_left_foot(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
        if(sp_parameter->visualize_movement_){
            display_pose(pose, parameter_.left_foot_frame_, ros_publisher_left_foot_
                         , {0, 1, 0});
        }

        if(sp_parameter->visualize_left_foot_arrows_){
            display_pose_as_arrows(pose, parameter_.left_foot_frame_, ros_publisher_left_foot_arrow_
                                   , {1, 1, 1}, {0, 0, 0}
                                   , sp_parameter->visualize_arrows_use_gradient_);
        }
    }

    void ThrowVisualizer::display_right_foot(std::shared_ptr<bitbots_splines::PoseHandle> & pose){
        if(sp_parameter->visualize_movement_){
            display_pose(pose, parameter_.right_foot_frame_, ros_publisher_right_foot_
                         , {0, 0, 1});
        }

        if(sp_parameter->visualize_right_foot_arrows_){
            display_pose_as_arrows(pose, parameter_.right_foot_frame_, ros_publisher_right_foot_arrow_
                                   , {1, 1, 1}, {0, 0, 0}
                                   , sp_parameter->visualize_arrows_use_gradient_);
        }
    }

    void ThrowVisualizer::display_pose(std::shared_ptr<bitbots_splines::PoseHandle> & pose
                                       , const std::string & frame
                                       , ros::Publisher const & publisher
                                       , const Color & color){
        auto path = get_path(*pose, frame, sp_parameter->visualization_smoothness_);

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
        std::vector<double> times;

        if(sp_parameter->visualize_only_points_arrows_){
            for(auto & point : pose->x()->points()){
                times.emplace_back(point.time_);
            }
        }else{
            double minimum = pose->x()->min();
            double maximum = pose->x()->max();
            double time_step = (maximum - minimum) / sp_parameter->visualize_arrows_smoothness_;

            for(auto i = minimum; i <= maximum; i += time_step){
                times.emplace_back(i);
            }
        }

        display_pose_as_arrows(pose, frame, publisher, times, color_1, color_2, use_gradient);
    }

    void ThrowVisualizer::display_pose_as_arrows(std::shared_ptr<bitbots_splines::PoseHandle> & pose
                                                 , std::string const & frame
                                                 , ros::Publisher const & publisher
                                                 , const std::vector<double> & pose_times
                                                 , const Color & color_1
                                                 , const Color & color_2
                                                 , const bool & use_gradient){
        uint32_t index = 0;
        Color color_helper = color_1;
        Color color_step = use_gradient ? (color_2 - color_1) / pose_times.size() : Color(1.0, 0.0, 0.0);

        for(auto t : pose_times){
            auto arrow = get_arrow(pose->get_geometry_msg_pose(t), frame);
            arrow.ns = "throw_debug";
            arrow.id = index;
            ++index;

            arrow.color.r = color_helper.red_;
            arrow.color.g = color_helper.green_;
            arrow.color.b = color_helper.blue_;

            if(use_gradient){
                color_helper += color_step;
            }else{
                color_helper = color_step.red_ == 1.0 ? color_1 : color_2;
                color_step.red_ *= -1;
            }

            publisher.publish(arrow);
        }
    }

    visualization_msgs::Marker ThrowVisualizer::get_path(bitbots_splines::PoseHandle &pose
                                                         , const std::string &frame
                                                         , const double smoothness){
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.lifetime = ros::Duration(1000);
        marker.frame_locked = false;
        marker.header.frame_id = frame;
        marker.header.stamp = ros::Time::now();
        marker.pose.orientation.w = 1;
        marker.scale.x = 0.01;
        marker.color.a = 1;

        // Take length of spline, assuming values at the beginning and end are set for x
        double first_time = pose.x()->min();
        double last_time = pose.x()->max();

        if(smoothness > 1){
            for(double i = first_time; i <= last_time; i += (last_time - first_time) / smoothness) {
                geometry_msgs::Point point;
                point.x = pose.x()->position(i);
                point.y = pose.y()->position(i);
                point.z = pose.z()->position(i);

                marker.points.push_back(point);
            }
        }else{
            for(int i = 0; i < pose.x()->points().size(); ++i){
                geometry_msgs::Point point;
                point.x = pose.x()->points().at(i).position_;
                point.y = pose.y()->points().at(i).position_;
                point.z = pose.z()->points().at(i).position_;

                marker.points.push_back(point);
            }
        }

        return marker;
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
