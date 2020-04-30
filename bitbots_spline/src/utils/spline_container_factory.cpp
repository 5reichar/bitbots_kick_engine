#include "utils/spline_container_factory.h"
#include "spline/linear_spline.h"
#include "spline/cubic_spline.h"
#include "spline/smooth_spline.h"
#include "spline/beziercurve.h"

namespace bitbots_splines
{
    SplineContainer SplineContainerFactory::create_linear_spline_container()
    {
        SplineContainer traj;

        traj.add(CurvePurpose::is_double_support, new LinearSpline());
        traj.add(CurvePurpose::is_left_support_foot, new LinearSpline());
        traj.add(CurvePurpose::trunk_position_x, new LinearSpline());
        traj.add(CurvePurpose::trunk_position_y, new LinearSpline());
        traj.add(CurvePurpose::trunk_position_z, new LinearSpline());
        traj.add(CurvePurpose::trunk_axis_x, new LinearSpline());
        traj.add(CurvePurpose::trunk_axis_y, new LinearSpline());
        traj.add(CurvePurpose::trunk_axis_z, new LinearSpline());
        traj.add(CurvePurpose::foot_position_x, new LinearSpline());
        traj.add(CurvePurpose::foot_position_y, new LinearSpline());
        traj.add(CurvePurpose::foot_position_z, new LinearSpline());
        traj.add(CurvePurpose::foot_axis_x, new LinearSpline());
        traj.add(CurvePurpose::foot_axis_y, new LinearSpline());
        traj.add(CurvePurpose::foot_axis_z, new LinearSpline());

        return traj;
    }

    SplineContainer SplineContainerFactory::create_cubic_spline_container()
    {
        SplineContainer traj;

        traj.add(CurvePurpose::is_double_support, new CubicSpline());
        traj.add(CurvePurpose::is_left_support_foot, new CubicSpline());
        traj.add(CurvePurpose::trunk_position_x, new CubicSpline());
        traj.add(CurvePurpose::trunk_position_y, new CubicSpline());
        traj.add(CurvePurpose::trunk_position_z, new CubicSpline());
        traj.add(CurvePurpose::trunk_axis_x, new CubicSpline());
        traj.add(CurvePurpose::trunk_axis_y, new CubicSpline());
        traj.add(CurvePurpose::trunk_axis_z, new CubicSpline());
        traj.add(CurvePurpose::foot_position_x, new CubicSpline());
        traj.add(CurvePurpose::foot_position_y, new CubicSpline());
        traj.add(CurvePurpose::foot_position_z, new CubicSpline());
        traj.add(CurvePurpose::foot_axis_x, new CubicSpline());
        traj.add(CurvePurpose::foot_axis_y, new CubicSpline());
        traj.add(CurvePurpose::foot_axis_z, new CubicSpline());

        return traj;
    }

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

    SplineContainer SplineContainerFactory::create_beziercurve_container()
    {
        SplineContainer traj;

        traj.add(CurvePurpose::is_double_support, new Beziercurve());
        traj.add(CurvePurpose::is_left_support_foot, new Beziercurve());
        traj.add(CurvePurpose::trunk_position_x, new Beziercurve());
        traj.add(CurvePurpose::trunk_position_y, new Beziercurve());
        traj.add(CurvePurpose::trunk_position_z, new Beziercurve());
        traj.add(CurvePurpose::trunk_axis_x, new Beziercurve());
        traj.add(CurvePurpose::trunk_axis_y, new Beziercurve());
        traj.add(CurvePurpose::trunk_axis_z, new Beziercurve());
        traj.add(CurvePurpose::foot_position_x, new Beziercurve());
        traj.add(CurvePurpose::foot_position_y, new Beziercurve());
        traj.add(CurvePurpose::foot_position_z, new Beziercurve());
        traj.add(CurvePurpose::foot_axis_x, new Beziercurve());
        traj.add(CurvePurpose::foot_axis_y, new Beziercurve());
        traj.add(CurvePurpose::foot_axis_z, new Beziercurve());

        return traj;
    }

}