#include "KickEngineNode.hpp"
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <bitbots_msgs/JointCommand.h>
#include <humanoid_league_msgs/RobotControlState.h>
#include <std_msgs/Char.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <bitbots_kick_engine/WalkingDebug.h>

KickEngineNode::KickEngineNode()
{
    initialise_ros_subcribtions();
    initialise_ros_publisher();

    robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
    robot_model_loader.loadKinematicsSolvers(
        kinematics_plugin_loader::KinematicsPluginLoaderPtr(
            new kinematics_plugin_loader::KinematicsPluginLoader()));

    m_kinematic_model = robot_model_loader.getModel();

    m_goal_state.reset(new robot_state::RobotState(m_kinematic_model));
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

    m_current_state.reset(new robot_state::RobotState(m_kinematic_model));
    m_current_state->setToDefaultValues();

    m_bio_ik_solver = bitbots_ik::BioIKSolver(*m_kinematic_model->getJointModelGroup("All"),
                                              *m_kinematic_model->getJointModelGroup("LeftLeg"),
                                              *m_kinematic_model->getJointModelGroup("RightLeg"));
    m_bio_ik_solver.set_use_approximate(true);
}

void KickEngineNode::kick(geometry_msgs::Vector3 & ball_position, geometry_msgs::Vector3 & target_position)
{
    m_kick_engine.kick(ball_position, target_position);

    publish_kick();
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

    m_ros_publisher_debug = m_ros_node_handle.advertise<bitbots_kick_engine::WalkingDebug>("kick_debug", 1);
    m_ros_publisher_debug_marker = m_ros_node_handle.advertise<visualization_msgs::Marker>("kick_debug_marker", 1);
}

void KickEngineNode::kick_ball(geometry_msgs::Vector3 & ball_position, geometry_msgs::Vector3 & target_position)
{
    uint16_t odometry_counter = 1;
    ros::Rate loopRate(10);

    while (ros::ok())
    {
        if (m_kick_engine.has_new_goals())
        {
            kick();
        }

        if (odometry_counter > m_uint_odometry_publish_factor)
        {
            publish_odemetry();
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

void KickEngineNode::publish_kick()
{
    // TODO implementation
    auto goal_state = m_kick_engine.get_goal_state();

    if (m_node_service.convert_goal_coordinate_from_support_foot_to_trunk_based(m_kick_engine, goal_state))
    {
        publish_controler_commands(m_kick_engine.get_joint_names(), m_node_service.get_joint_goals(goal_state));
    }
    

    // TODO Publish current support state
    m_ros_publisher_support.publish(m_kick_engine.get_support_foot_state())

    if (m_bool_debug)
    {
        publish_debug();
        publish_marker();
    }
}

void KickEngineNode::publish_controler_commands(std::vector<std::string> joint_names, std::vector<double> positions)
{
    std::vector<double> ones(joint_names.size(), -1.0);
    publish_controler_commands(joint_names, positions, ones, ones, ones);
}

void KickEngineNode::publish_controler_commands(std::vector<std::string> joint_names, std::vector<double> positions, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> max_currents)
{
    bitbots_msgs::JointCommand joint_command_msg;
    
    joint_command_msg.header.stamp = ros::Time::now();
    joint_command_msg.joint_names = joint_names;
    joint_command_msg.positions = positions;
    joint_command_msg.velocities = velocities;
    joint_command_msg.accelerations = accelerations;
    joint_command_msg.max_currents = max_currents;

    m_ros_publisher_controller_command.publish(joint_command_msg);

}

void KickEngineNode::publish_odemetry()
{
    ros::Time current_time = ros::Time::now();
    geometry_msgs::Quaternion quaternion_msg;
    tf::Vector3 position;

    m_node_service.get_odemetry_data(m_kick_engine, position, quaternion_msg)

    tf::TransformBroadcaster odometry_broadcaster;
    geometry_msgs::TransformStamped odometry_transformation;

    odometry_transformation = geometry_msgs::TransformStamped();
    odometry_transformation.header.stamp = current_time;
    odometry_transformation.header.frame_id = "odom";
    odometry_transformation.child_frame_id = "base_link";
    odometry_transformation.transform.translation.x = position[0];
    odometry_transformation.transform.translation.y = position[1];
    odometry_transformation.transform.translation.z = position[2];
    odometry_transformation.transform.rotation = quaternion_msg;
    
    odometry_broadcaster.sendTransform(odometry_transformation);

    // send the odometry also as message
    nav_msgs::Odometry odometry_nav_msgs;

    odometry_nav_msgs.header.stamp = current_time;
    odometry_nav_msgs.header.frame_id = "odom";
    odometry_nav_msgs.child_frame_id = "base_link";
    odometry_nav_msgs.pose.pose.position.x = position[0];
    odometry_nav_msgs.pose.pose.position.y = position[1];
    odometry_nav_msgs.pose.pose.position.z = position[2];
    odometry_nav_msgs.pose.pose.orientation = quaternion_msg;
    odometry_nav_msgs.twist.twist = m_kick_engine.get_twist();

    m_ros_publisher_odometry.publish(_odom_msg);
}

void KickEngineNode::publish_debug()
{
    // TODO implementation
}

void KickEngineNode::publish_marker()
{
    // TODO implementation
}

void KickEngineNode::robot_state_callback(const humanoid_league_msgs::RobotControlState msg)
{
    m_kick_engine.set_robot_state(msg);
}

void KickEngineNode::kick_callback(const humanoid_league_msgs::Kick action)
{
    kick_ball(action.ball_pos, action.target);
}

int main(int argc, char **argv)
{
}