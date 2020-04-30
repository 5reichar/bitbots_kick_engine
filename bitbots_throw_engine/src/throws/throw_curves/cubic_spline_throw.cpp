#include "throws/throw_curves/cubic_spline_throw.h"
#include "../../bitbots_spline/include/spline/cubic_spline.hpp"

CubicSplineThrow::CubicSplineThrow()
{
	sp_spline_container_->add(bitbots_splines::CurvePurpose::trunk_position_x, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::trunk_position_y, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::trunk_position_z, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::trunk_axis_x, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::trunk_axis_y, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::trunk_axis_z, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::left_hand_position_x, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::left_hand_position_y, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::left_hand_position_z, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::left_hand_axis_x, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::left_hand_axis_y, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::left_hand_axis_z, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::right_hand_position_x, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::right_hand_position_y, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::right_hand_position_z, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::right_hand_axis_x, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::right_hand_axis_y, new bitbots_splines::CubicSpline());
	sp_spline_container_->add(bitbots_splines::CurvePurpose::right_hand_axis_z, new bitbots_splines::CubicSpline());
}