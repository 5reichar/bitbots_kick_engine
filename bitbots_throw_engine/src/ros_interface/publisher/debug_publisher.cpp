#include "ros_interface/publisher/debug_publisher.h"
#include <visualization_msgs/Marker.h>
#include <bitbots_throw/throw_debug.h>
#include "ros_interface/publisher/system_publisher.h"

namespace bitbots_throw{
    DebugPublisher::DebugPublisher(ros::NodeHandle & ros_node_handle
                                  ,std::string const & topic
                                  ,std::string const & topic_marker){
        marker_id_ = 0;
        ros_publisher_debug_ = ros_node_handle.advertise<bitbots_throw::throw_debug>(topic, 1);
        ros_publisher_debug_marker_ = ros_node_handle.advertise<visualization_msgs::Marker>(topic_marker, 1);
    }

    void DebugPublisher::print_throw_points(std::string point_data){
        SystemPublisher publisher;
        publisher.print_to_file(point_data, "output.csv");
    }

    void DebugPublisher::publish_ik_debug(ThrowResponse const & response, int8_t const & percentage_done, int8_t const & movement_stage){
        //only do something if someone is listing
        if (ros_publisher_debug_.getNumSubscribers() == 0 && ros_publisher_debug_marker_.getNumSubscribers() == 0){
            return;
        }

        bitbots_throw::throw_debug msg;

        msg.percentage_done = percentage_done;
        msg.movement_stage = movement_stage;
        tf2::toMsg(response.support_foot_to_left_hand_, msg.left_hand_goal);
        tf2::toMsg(response.support_foot_to_right_hand_, msg.right_hand_goal);
        tf2::toMsg(response.support_foot_to_left_foot_, msg.left_foot_goal);
        tf2::toMsg(response.support_foot_to_right_foot_, msg.right_foot_goal);

        publish_arrow_marker("throw_engine_left_hand_goal"
                            ,"base_link"
                            ,msg.left_hand_goal
                            ,255
                            ,127
                            ,0
                            ,1);
        publish_arrow_marker("throw_engine_right_hand_goal"
                            ,"base_link"
                            ,msg.right_hand_goal
                            ,255
                            ,127
                            ,0
                            ,1);
        publish_arrow_marker("throw_engine_left_foot_goal"
                            ,"base_link"
                            ,msg.left_foot_goal
                            ,255
                            ,127
                            ,0
                            ,1);
        publish_arrow_marker("throw_engine_right_foot_goal"
                            ,"base_link"
                            ,msg.right_foot_goal
                            ,255
                            ,127
                            ,0
                            ,1);

        ros_publisher_debug_.publish(msg);
    }

    void DebugPublisher::publish_arrow_marker(std::string const & name_space
                                             ,std::string const & frame
                                             ,geometry_msgs::Pose const & pose
                                             ,float const & r
                                             ,float const & g
                                             ,float const & b
                                             ,float const & a){
        visualization_msgs::Marker marker_msg;
        marker_msg.header.stamp = ros::Time::now();
        marker_msg.header.frame_id = frame;

        marker_msg.type = marker_msg.ARROW;
        marker_msg.ns = name_space;
        marker_msg.action = marker_msg.ADD;
        marker_msg.pose = pose;

        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        marker_msg.color = color;

        geometry_msgs::Vector3 scale;
        scale.x = 0.01;
        scale.y = 0.003;
        scale.z = 0.003;
        marker_msg.scale = scale;

        marker_msg.id = marker_id_;
        marker_id_++;

        ros_publisher_debug_marker_.publish(marker_msg);
    }
} //bitbots_throw