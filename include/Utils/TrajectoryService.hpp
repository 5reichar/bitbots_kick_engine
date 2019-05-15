/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef TRAJECTORYSERVICE_HPP
#define TRAJECTORYSERVICE_HPP

#include "../bitbots_splines/Curve.hpp"
#include "SplineContainer.hpp"
#include <eigen3/Eigen/Dense>

namespace bitbots_splines
{

class TrajectoryService
{
public:
    /**
     * Simple typedef for trajectories container
     */
    typedef SplineContainer<Curve> Trajectories;

    /**
     * Return initialized trajectories for
     * trunk/foot ik cartesian with empty splines
     */
    static Trajectories TrajectoriesInit();

    /**
     * Compute from given spline container
     * trajectory Cartesian trunk and foot
     * position/velocity/acceleration
     * and assign it to given vector
     */
    static void TrajectoriesTrunkFootPos(
        double t, const Trajectories &traj,
        Eigen::Vector3d &trunkPos,
        Eigen::Vector3d &trunkAxis,
        Eigen::Vector3d &footPos,
        Eigen::Vector3d &footAxis);

    static void TrajectoriesTrunkFootVel(
        double t, const Trajectories &traj,
        Eigen::Vector3d &trunkPosVel,
        Eigen::Vector3d &trunkAxisVel,
        Eigen::Vector3d &footPosVel,
        Eigen::Vector3d &footAxisVel);

    static void TrajectoriesTrunkFootAcc(
        double t, const Trajectories &traj,
        Eigen::Vector3d &trunkPosAcc,
        Eigen::Vector3d &trunkAxisAcc,
        Eigen::Vector3d &footPosAcc,
        Eigen::Vector3d &footAxisAcc);

    static void TrajectoriesSupportFootState(
        double t, const Trajectories &traj,
        bool &isDoubleSupport,
        bool &isLeftsupportFoot);

    /**
     * Default Cartesian state check function.
     * Return positive cost value
     * if given time and Cartesian state are outside
     * standard valid range
     */
    static double DefaultCheckState(
        const Eigen::VectorXd &params,
        double t,
        const Eigen::Vector3d &trunkPos,
        const Eigen::Vector3d &trunkAxis,
        const Eigen::Vector3d &footPos,
        const Eigen::Vector3d &footAxis);

private:
};

} // namespace bitbots_splines

#endif
