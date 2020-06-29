#ifndef BITBOTS_THROW_CONTROLER_COMMAND_PUBLISHER_H
#define BITBOTS_THROW_CONTROLER_COMMAND_PUBLISHER_H

#include "ros/ros.h"
#include "bitbots_splines/abstract_ik.h"

namespace bitbots_throw{
    class ControllerCommandPublisher{
    public:
        ControllerCommandPublisher(ros::NodeHandle & ros_node_handle, std::string topic = "throw_motor_goals");

        virtual void publish(ros::Time time
                            ,bitbots_splines::JointGoals joint_goals);
        virtual void publish(ros::Time time
                            ,std::vector<std::string> joint_names
                            ,std::vector<double> positions);
        virtual void publish(ros::Time time
                            ,std::vector<std::string> joint_names
                            ,std::vector<double> positions
                            ,std::vector<double> velocities);
        virtual void publish(ros::Time time
                            ,std::vector<std::string> joint_names
                            ,std::vector<double> positions
                            ,std::vector<double> velocities
                            ,std::vector<double> accelerations);
        virtual void publish(ros::Time time
                            ,std::vector<std::string> joint_names
                            ,std::vector<double> positions
                            ,std::vector<double> velocities
                            ,std::vector<double> accelerations
                            ,std::vector<double> max_currents);

    private:
        ros::Publisher ros_publisher_controller_command_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_CONTROLER_COMMAND_PUBLISHER_H