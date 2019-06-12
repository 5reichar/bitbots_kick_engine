#include "KickEngineNode.hpp"
#include <geometry_msgs/Vector3.h>

KickEngineNode::KickEngineNode()
{
    initialise_ros_subcribtions();
    initialise_ros_publisher();

    robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
    robot_model_loader.loadKinematicsSolvers(
        kinematics_plugin_loader::KinematicsPluginLoaderPtr(
            new kinematics_plugin_loader::KinematicsPluginLoader()));

    m_kinematic_model = robot_model_loader.getModel();

    m_goal_state.reset(new robot_state::RobotState(_kinematic_model));
    m_goal_state->setToDefaultValues();
    // we have to set some good initial position in the goal state, since we are using a gradient
    // based method. Otherwise, the first step will be not correct
    std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
    std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
    for (int i = 0; i < names_vec.size(); i++)
    {
        // besides its name, this method only changes a single joint position...
        m_goal_state->setJointPositions(names_vec[i], &pos_vec[i]);
    }

    m_current_state.reset(new robot_state::RobotState(_kinematic_model));
    m_current_state->setToDefaultValues();

    m_bio_ik_solver = bitbots_ik::BioIKSolver(*m_kinematic_model->getJointModelGroup("All"),
                                              *m_kinematic_model->getJointModelGroup("LeftLeg"),
                                              *m_kinematic_model->getJointModelGroup("RightLeg"));
    m_bio_ik_solver.set_use_approximate(true);
}

void KickEngineNode::kick()
{
    // TODO implementation
}

void KickEngineNode::initialise_ros_subcribtions()
{
    // TODO implementation
    m_ros_subsciber_kick = m_ros_node_handle.subscribe("kick", 1, &KickEngineNode::kick_callback, this, ros::TransportHints().tcpNoDelay());
    m_ros_subsciber_robot_state = m_ros_node_handle.subscribe("robot_state", 1, &KickEngineNode::robot_state_callback, this, ros::TransportHints().tcpNoDelay());
}

void KickEngineNode::initialise_ros_publisher()
{
    // TODO implementation
    m_ros_publisher_controller_command = m_ros_node_handle.advertise<bitbots_msgs::JointCommand>("kick_motor_goals", 1);
    m_ros_publisher_odometry = m_ros_node_handle.advertise<nav_msgs::Odometry>("kick_odometry", 1);
    m_ros_publisher_support = m_ros_node_handle.advertise<std_msgs::Char>("kick_support_state", 1);

    m_ros_publisher_debug = m_ros_node_handle.advertise<bitbots_quintic_walk::WalkingDebug>("kick_debug", 1);
    m_ros_publisher_debug_marker = m_ros_node_handle.advertise<visualization_msgs::Marker>("kick_debug_marker", 1);
}

void KickEngineNode::kick_ball(geometry_msgs::Vector3 const &const ball_position, geometry_msgs::Vector3 const &const target_position)
{
    // TODO implementation

    uint16_t odometry_counter = 1;

    while (ros::ok())
    {
        ros::Rate loopRate();

        if (m_kick_engine.has_new_goals())
        {
            kick();
        }

        if (odometry_counter > m_uint_odometry_publish_factor)
        {
            publish_odometry();
            odometry_counter = 1;
        }
        else
        {
            ++odometry_counter;
        }

        ros::spinOnce();
        loopRate.sleep();
    }
}

void publish_controler_commands(std::vector<std::string> joint_names, std::vector<double> positions) const
{
    // TODO implementation
}

void KickEngineNode::publish_kick() const
{
    // TODO implementation

    publish_controler_commands(m_kick_engine.get_joint_names(), m_kick_engine.get_joint_goals());

    // TODO Publish current support state

    if (m_bool_debug)
    {
        publish_debug();
        publish_marker();
    }
}

void KickEngineNode::publish_odometry() const
{
    // TODO implementation
}

void KickEngineNode::robot_state_callback(const humanoid_league_msgs::RobotControlState msg)
{
    m_kick_engine.set_robot_state(msg);
}

void KickEngineNode::kick_callback()
{
    // TODO implementation
}

int main(int argc, char **argv)
{
}