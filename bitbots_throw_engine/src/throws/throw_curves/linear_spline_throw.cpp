#include "throws/throw_curves/linear_spline_throw.h"
#include "../../bitbots_splines_extension/include/spline/linear_spline.h"

LinearSplineThrow::LinearSplineThrow()
	:ThrowCurve
	(
		std::make_shared<bitbots_splines::PoseHandle> // Left Hand
		(
			std::make_shared<bitbots_splines::LinearSpline>(), // x
			std::make_shared<bitbots_splines::LinearSpline>(), // y
			std::make_shared<bitbots_splines::LinearSpline>(), // z
			std::make_shared<bitbots_splines::LinearSpline>(), // roll
			std::make_shared<bitbots_splines::LinearSpline>(), // pitch
			std::make_shared<bitbots_splines::LinearSpline>()  // yaw
		),
		std::make_shared<bitbots_splines::PoseHandle> // Right Hand
		(
			std::make_shared<bitbots_splines::LinearSpline>(), // x
			std::make_shared<bitbots_splines::LinearSpline>(), // y
			std::make_shared<bitbots_splines::LinearSpline>(), // z
			std::make_shared<bitbots_splines::LinearSpline>(), // roll
			std::make_shared<bitbots_splines::LinearSpline>(), // pitch
			std::make_shared<bitbots_splines::LinearSpline>()  // yaw
		),
		std::make_shared<bitbots_splines::PoseHandle> // Trunk
		(
			std::make_shared<bitbots_splines::LinearSpline>(), // x
			std::make_shared<bitbots_splines::LinearSpline>(), // y
			std::make_shared<bitbots_splines::LinearSpline>(), // z
			std::make_shared<bitbots_splines::LinearSpline>(), // roll
			std::make_shared<bitbots_splines::LinearSpline>(), // pitch
			std::make_shared<bitbots_splines::LinearSpline>()  // yaw
		)
	)
{
}