#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

void KickEngineNode::kick()
{
    // TODO implementation
}

void KickEngineNode::initialise_ros_subcribtions()
{
    // TODO implementation
}

void KickEngineNode::initialise_ros_publisher()
{
    // TODO implementation
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

void KickEngineNode::publish_kick() const
{
    // TODO implementation

    publish_controler_commands(m_kick_engine.get_joint_names(), m_kick_engine.get_joint_goals()):

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
    // TODO implementation
}

void KickEngineNode::kick_callback()
{
    // TODO implementation
}


void KickEngineNode::velocity_cammand_callback(const geometry_msgs::Twist msg)
{
    // TODO implementation
}

int main(int argc, char **argv)
{
}