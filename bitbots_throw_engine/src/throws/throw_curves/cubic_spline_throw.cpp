#include "throws/throw_curves/cubic_spline_throw.h"
#include "../../bitbots_spline/include/spline/cubic_spline.h"

CubicSplineThrow::CubicSplineThrow()
	:ThrowCurve
	(
		std::make_shared<bitbots_splines::PoseHandle> // Left Hand
		(
			std::make_shared<bitbots_splines::CubicSpline>(), // x
			std::make_shared<bitbots_splines::CubicSpline>(), // y
			std::make_shared<bitbots_splines::CubicSpline>(), // z
			std::make_shared<bitbots_splines::CubicSpline>(), // roll
			std::make_shared<bitbots_splines::CubicSpline>(), // pitch
			std::make_shared<bitbots_splines::CubicSpline>()  // yaw
		),
		std::make_shared<bitbots_splines::PoseHandle> // Right Hand
		(
			std::make_shared<bitbots_splines::CubicSpline>(), // x
			std::make_shared<bitbots_splines::CubicSpline>(), // y
			std::make_shared<bitbots_splines::CubicSpline>(), // z
			std::make_shared<bitbots_splines::CubicSpline>(), // roll
			std::make_shared<bitbots_splines::CubicSpline>(), // pitch
			std::make_shared<bitbots_splines::CubicSpline>()  // yaw
		),
		std::make_shared<bitbots_splines::PoseHandle> // Trunk
		(
			std::make_shared<bitbots_splines::CubicSpline>(), // x
			std::make_shared<bitbots_splines::CubicSpline>(), // y
			std::make_shared<bitbots_splines::CubicSpline>(), // z
			std::make_shared<bitbots_splines::CubicSpline>(), // roll
			std::make_shared<bitbots_splines::CubicSpline>(), // pitch
			std::make_shared<bitbots_splines::CubicSpline>()  // yaw
		)
	)
{
}