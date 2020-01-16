#include "utils/SplineContainerFactory.hpp"
#include "spline/SmoothSpline.hpp"

namespace bitbots_splines
{
    SplineContainer SplineContainerFactory::create_smooth_spline_container()
    {
        SplineContainer traj;

        traj.add(CurvePurpose::is_double_support, new SmoothSpline());
        traj.add(CurvePurpose::is_left_support_foot, new SmoothSpline());
        traj.add(CurvePurpose::trunk_position_x, new SmoothSpline());
        traj.add(CurvePurpose::trunk_position_y, new SmoothSpline());
        traj.add(CurvePurpose::trunk_position_z, new SmoothSpline());
        traj.add(CurvePurpose::trunk_axis_x, new SmoothSpline());
        traj.add(CurvePurpose::trunk_axis_y, new SmoothSpline());
        traj.add(CurvePurpose::trunk_axis_z, new SmoothSpline());
        traj.add(CurvePurpose::foot_position_x, new SmoothSpline());
        traj.add(CurvePurpose::foot_position_y, new SmoothSpline());
        traj.add(CurvePurpose::foot_position_z, new SmoothSpline());
        traj.add(CurvePurpose::foot_axis_x, new SmoothSpline());
        traj.add(CurvePurpose::foot_axis_y, new SmoothSpline());
        traj.add(CurvePurpose::foot_axis_z, new SmoothSpline());

        return traj;
    }

}