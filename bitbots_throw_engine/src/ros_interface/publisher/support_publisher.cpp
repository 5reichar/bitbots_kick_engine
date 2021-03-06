#include "ros_interface/publisher/support_publisher.h"
#include <std_msgs/Char.h>

namespace bitbots_throw{
    SupportPublisher::SupportPublisher(ros::NodeHandle & ros_node_handle, std::string topic){
        current_support_foot_ = '\0';
        ros_publisher_support_ = ros_node_handle.advertise<std_msgs::Char>(topic, 1);
    }

    void SupportPublisher::publish(){
        publish('d');
    }

    void SupportPublisher::publish(char const support_foot){
        // publish if foot changed
        if(support_foot != current_support_foot_)
        {
            std_msgs::Char support_state;

            support_state.data = support_foot;
            current_support_foot_ = support_foot;

            ros_publisher_support_.publish(support_state);
        }
    }
} //bitbots_throw