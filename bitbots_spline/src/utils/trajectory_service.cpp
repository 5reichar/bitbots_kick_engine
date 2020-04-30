/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/

#include "utils/trajectory_service.h"
#include "utils/spline_container_factory.h"

namespace bitbots_splines
{

SplineContainer TrajectoryService::TrajectoriesInit()
{
    return SplineContainerFactory::create_smooth_spline_container();
}

void TrajectoryService::TrajectoriesTrunkFootPos(
    double t,
    const SplineContainer &traj,
    Eigen::Vector3d &trunkPos,
    Eigen::Vector3d &trunkAxis,
    Eigen::Vector3d &footPos,
    Eigen::Vector3d &footAxis)
{
    //Compute Cartesian positions
	trunkPos = GetTrajectoriePositionTrunk(t, traj);
	trunkAxis = GetTrajectorieAxisTrunk(t, traj);
	footPos = GetTrajectoriePositionFoot(t, traj);
	footAxis = GetTrajectorieAxisFoot(t, traj);
}
Eigen::Vector3d& TrajectoryService::GetTrajectoriePositionTrunk(double time, const SplineContainer& trajectories)
{
	auto return_value = Eigen::Vector3d(
                            trajectories.get(CurvePurpose::trunk_position_x)->pos(time),
                            trajectories.get(CurvePurpose::trunk_position_y)->pos(time),
                            trajectories.get(CurvePurpose::trunk_position_z)->pos(time));
    return return_value;
}
Eigen::Vector3d& TrajectoryService::GetTrajectorieAxisTrunk(double time, const SplineContainer& trajectories)
{
	auto return_value = Eigen::Vector3d(
                            trajectories.get(CurvePurpose::trunk_axis_x)->pos(time),
                            trajectories.get(CurvePurpose::trunk_axis_y)->pos(time),
                            trajectories.get(CurvePurpose::trunk_axis_z)->pos(time));
    return return_value;
}
Eigen::Vector3d& TrajectoryService::GetTrajectoriePositionFoot(double time, const SplineContainer& trajectories)
{
	auto return_value = Eigen::Vector3d(
                            trajectories.get(CurvePurpose::foot_position_x)->pos(time),
                            trajectories.get(CurvePurpose::foot_position_y)->pos(time),
                            trajectories.get(CurvePurpose::foot_position_z)->pos(time));
    return return_value;
}
Eigen::Vector3d& TrajectoryService::GetTrajectorieAxisFoot(double time, const SplineContainer& trajectories)
{
	auto return_value = Eigen::Vector3d(
                            trajectories.get(CurvePurpose::foot_axis_x)->pos(time),
                            trajectories.get(CurvePurpose::foot_axis_y)->pos(time),
                            trajectories.get(CurvePurpose::foot_axis_z)->pos(time));
    return return_value;
}
void TrajectoryService::TrajectoriesTrunkFootVel(
    double t,
    const SplineContainer &traj,
    Eigen::Vector3d &trunkPosVel,
    Eigen::Vector3d &trunkAxisVel,
    Eigen::Vector3d &footPosVel,
    Eigen::Vector3d &footAxisVel)
{
    //Compute Cartesian velocities
    trunkPosVel = Eigen::Vector3d(
        traj.get(CurvePurpose::trunk_position_x)->vel(t),
        traj.get(CurvePurpose::trunk_position_y)->vel(t),
        traj.get(CurvePurpose::trunk_position_z)->vel(t));
    trunkAxisVel = Eigen::Vector3d(
        traj.get(CurvePurpose::trunk_axis_x)->vel(t),
        traj.get(CurvePurpose::trunk_axis_y)->vel(t),
        traj.get(CurvePurpose::trunk_axis_z)->vel(t));
    footPosVel = Eigen::Vector3d(
        traj.get(CurvePurpose::foot_position_x)->vel(t),
        traj.get(CurvePurpose::foot_position_y)->vel(t),
        traj.get(CurvePurpose::foot_position_z)->vel(t));
    footAxisVel = Eigen::Vector3d(
        traj.get(CurvePurpose::foot_axis_x)->vel(t),
        traj.get(CurvePurpose::foot_axis_y)->vel(t),
        traj.get(CurvePurpose::foot_axis_z)->vel(t));
}
void TrajectoryService::TrajectoriesTrunkFootAcc(
    double t,
    const SplineContainer &traj,
    Eigen::Vector3d &trunkPosAcc,
    Eigen::Vector3d &trunkAxisAcc,
    Eigen::Vector3d &footPosAcc,
    Eigen::Vector3d &footAxisAcc)
{
    //Compute Cartesian accelerations
    trunkPosAcc = Eigen::Vector3d(
        traj.get(CurvePurpose::trunk_position_x)->acc(t),
        traj.get(CurvePurpose::trunk_position_y)->acc(t),
        traj.get(CurvePurpose::trunk_position_z)->acc(t));
    trunkAxisAcc = Eigen::Vector3d(
        traj.get(CurvePurpose::trunk_axis_x)->acc(t),
        traj.get(CurvePurpose::trunk_axis_y)->acc(t),
        traj.get(CurvePurpose::trunk_axis_z)->acc(t));
    footPosAcc = Eigen::Vector3d(
        traj.get(CurvePurpose::foot_position_x)->acc(t),
        traj.get(CurvePurpose::foot_position_y)->acc(t),
        traj.get(CurvePurpose::foot_position_z)->acc(t));
    footAxisAcc = Eigen::Vector3d(
        traj.get(CurvePurpose::foot_axis_x)->acc(t),
        traj.get(CurvePurpose::foot_axis_y)->acc(t),
        traj.get(CurvePurpose::foot_axis_z)->acc(t));
}
void TrajectoryService::TrajectoriesSupportFootState(
    double t,
    const SplineContainer &traj,
    bool &isDoubleSupport,
    bool &isLeftsupportFoot)
{
    //Compute support foot state
    isDoubleSupport = GetTrajectorieFootSupportDouble(t, traj);
    isLeftsupportFoot = GetTrajectorieFootSupportLeft(t, traj);
}
bool& TrajectoryService::GetTrajectorieFootSupportDouble(double time, const SplineContainer& trajectories)
{
    auto return_value = trajectories.get(CurvePurpose::is_double_support)->pos(time) >= 0.5 ? true : false;
    return return_value;
}
bool& TrajectoryService::GetTrajectorieFootSupportLeft(double time, const SplineContainer& trajectories)
{
    auto return_value = trajectories.get(CurvePurpose::is_left_support_foot)->pos(time) >= 0.5 ? true : false;
    return return_value;
}

double TrajectoryService::DefaultCheckState(
    const Eigen::VectorXd &params,
    double t,
    const Eigen::Vector3d &trunkPos,
    const Eigen::Vector3d &trunkAxis,
    const Eigen::Vector3d &footPos,
    const Eigen::Vector3d &footAxis)
{
    (void)params;
    (void)t;
    double cost = 0.0;
    if (trunkPos.z() < 0.0)
    {
        cost += 1000.0 - 1000.0 * trunkPos.z();
    }
    if (trunkAxis.norm() >= M_PI / 2.0)
    {
        cost += 1000.0 + 1000.0 * (trunkAxis.norm() - M_PI / 2.0);
    }
    if (fabs(footPos.y()) < 2.0 * 0.037)
    {
        cost += 1000.0 + 1000.0 * (2.0 * 0.037 - fabs(footPos.y()));
    }
    if (footPos.z() < -1e6)
    {
        cost += 1000.0 - 1000.0 * footPos.z();
    }
    if (trunkPos.z() - footPos.z() < 0.20)
    {
        cost += 1000.0 - 1000.0 * (trunkPos.z() - footPos.z() - 0.20);
    }
    if (footAxis.norm() >= M_PI / 2.0)
    {
        cost += 1000.0 + 1000.0 * (footAxis.norm() - M_PI / 2.0);
    }

    return cost;
}

} // namespace bitbots_splines
