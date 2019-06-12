#ifndef KICKENGINE_HPP
#define KICKENGINE_HPP

#include <humanoid_league_msgs/RobotControlState.h>

class KickEngine
{
public:
    enum class KickType : uint16_t
    {
        standard = 0
    };

    KickEngine(const humanoid_league_msgs::RobotControlState msg);

    void set_param();
    void set_robot_state(const humanoid_league_msgs::RobotControlState msg);
    void kick(geometry_msgs::Vector3 const &const ball_position, geometry_msgs::Vector3 const &const target_position);

    std::vector<std::string> get_joint_names() const;
    std::vector<double> get_joint_goals() const;
    bool has_new_goals() const;

private:
    geometry_msgs::Vector3 get_kick_start_position();
    void move_left_feet_to_position(geometry_msgs::Vector3 position);
    void move_right_feet_to_position(geometry_msgs::Vector3 position);
    void move_feet_to_position(geometry_msgs::Vector3 position);

    humanoid_league_msgs::RobotControlState::state m_robot_state;
};

#endif