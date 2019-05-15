#ifndef KICKENGINESERVICE_HPP
#define KICKENGINESERVICE_HPP

#include "bitbots_kick_engine/KickEngineBase.hpp"
#include <eigen3/Eigen/Dense>

class KickEngineService
{
public:
    KickEngineService();
    ~KickEngineService();

    void set_ball(double const x_position, double const y_position, double const z_position);
    void set_target(double const x_position, double const y_position, double const z_position);
    void calc_kick();

private:
    void calc_point_velocity_from_trajectory();
    std::vector<bitbots_splines::Curve::Point> get_kick_start_point();
    void calc_point_time_from_start_point(std::vector<bitbots_splines::Curve::Point> start_point);

    void computeCartesianPosition(Eigen::Vector3d &trunkPos,
                                  Eigen::Vector3d &trunkAxis,
                                  Eigen::Vector3d &footPos,
                                  Eigen::Vector3d &footAxis,
                                  bool &isLeftsupportFoot);

    void computeCartesianPositionAtTime(Eigen::Vector3d &trunkPos,
                                        Eigen::Vector3d &trunkAxis,
                                        Eigen::Vector3d &footPos,
                                        Eigen::Vector3d &footAxis,
                                        bool &isLeftsupportFoot,
                                        double time);

    // Ball kick checks
    bool check_kick_ball_to_the_left();
    bool check_kick_ball_to_the_right();
    bool check_can_kick_ball_to_target();
    bool check_use_left_leg_to_kick();

    KickEngineBase *m_p_kick_engine;
    std::vector<bitbots_splines::Curve::Point> m_vec_ball_position;
    std::vector<bitbots_splines::Curve::Point> m_vec_target_position;
};

#endif // KICKENGINESERVICE_HPP
