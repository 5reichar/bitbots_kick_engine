#include "bitbots_kick_engine/KickEngineService.hpp"

KickEngineService::KickEngineService()
{
}

KickEngineService::~KickEngineService()
{
    if (m_p_kick_engine != NULL)
    {
        delete m_p_kick_engine;
    }
}

void KickEngineService::set_ball(double const x_position, double const y_position, double const z_position)
{
    m_vec_ball_position.push_back(bitbots_splines::Curve::createPoint(0.0, x_position));
    m_vec_ball_position.push_back(bitbots_splines::Curve::createPoint(0.0, y_position));
    m_vec_ball_position.push_back(bitbots_splines::Curve::createPoint(0.0, z_position));
}

void KickEngineService::set_target(double const x_position, double const y_position, double const z_position)
{
    m_vec_target_position.push_back(bitbots_splines::Curve::createPoint(0.0, x_position));
    m_vec_target_position.push_back(bitbots_splines::Curve::createPoint(0.0, y_position));
    m_vec_target_position.push_back(bitbots_splines::Curve::createPoint(0.0, z_position));
}

void KickEngineService::calc_kick()
{
    calc_point_velocity_from_trajectory();
    auto kick_start_point = get_kick_start_point();
    // MABY_TODO: implementing here, move feet to kick position
    calc_point_time_from_start_point(kick_start_point);
}

void KickEngineService::calc_point_velocity_from_trajectory()
{
    double g = 9.81;
    double time_per_position_unit = 0.5; // TODO calculate from z axis and max/min/other height

    double distance_x = (m_vec_target_position[0].position - m_vec_ball_position[0].position);
    double time_x = distance_x * time_per_position_unit;
    double distance_y = (m_vec_target_position[1].position - m_vec_ball_position[1].position);
    double time_y = distance_y * time_per_position_unit;
}

std::vector<bitbots_splines::Curve::Point> KickEngineService::get_kick_start_point()
{
    std::vector<bitbots_splines::Curve::Point> start_point;

    start_point.push_back(bitbots_splines::Curve::createPoint(0.0, -1.0)); // TODO: change dummy values
    start_point.push_back(bitbots_splines::Curve::createPoint(0.0, 0.0));  // TODO: change dummy values
    start_point.push_back(bitbots_splines::Curve::createPoint(0.0, 2.0));  // TODO: change dummy values

    return start_point;
}

void KickEngineService::calc_point_time_from_start_point(std::vector<bitbots_splines::Curve::Point> start_point)
{
    double ball_hit_time;

    // TODO implement time calculation

    for (auto it : m_vec_ball_position)
    {
        it.time = ball_hit_time;
    }
}

void KickEngineService::computeCartesianPosition(Eigen::Vector3d &trunkPos,
                                                 Eigen::Vector3d &trunkAxis,
                                                 Eigen::Vector3d &footPos,
                                                 Eigen::Vector3d &footAxis,
                                                 bool &isLeftsupportFoot)
{
    // TODO implement
}

void KickEngineService::computeCartesianPositionAtTime(Eigen::Vector3d &trunkPos,
                                                       Eigen::Vector3d &trunkAxis,
                                                       Eigen::Vector3d &footPos,
                                                       Eigen::Vector3d &footAxis,
                                                       bool &isLeftsupportFoot,
                                                       double time)
{
    // TODO implement
}

bool KickEngineService::check_kick_ball_to_the_left()
{
    // TODO implement check
    return false;
}

bool KickEngineService::check_kick_ball_to_the_right()
{
    // TODO implement check
    return false;
}

bool KickEngineService::check_can_kick_ball_to_target()
{
    // TODO implement check
    return false;
}

bool KickEngineService::check_use_left_leg_to_kick()
{
    // TODO implement check
    return false;
}
