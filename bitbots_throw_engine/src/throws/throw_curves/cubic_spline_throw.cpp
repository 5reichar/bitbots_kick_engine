#include "throws/throw_curves/cubic_spline_throw.h"
#include "../../bitbots_splines_extension/include/spline/cubic_spline.h"

namespace bitbots_throw{
	CubicSplineThrow::CubicSplineThrow()
		:ThrowCurve(
			std::make_shared<bitbots_splines::PoseHandle>( // Left Hand
				std::make_shared<bitbots_splines::CubicSpline>(), // x
				std::make_shared<bitbots_splines::CubicSpline>(), // y
				std::make_shared<bitbots_splines::CubicSpline>(), // z
				std::make_shared<bitbots_splines::CubicSpline>(), // roll
				std::make_shared<bitbots_splines::CubicSpline>(), // pitch
				std::make_shared<bitbots_splines::CubicSpline>()  // yaw
			),
			std::make_shared<bitbots_splines::PoseHandle>( // Right Hand
				std::make_shared<bitbots_splines::CubicSpline>(), // x
				std::make_shared<bitbots_splines::CubicSpline>(), // y
				std::make_shared<bitbots_splines::CubicSpline>(), // z
				std::make_shared<bitbots_splines::CubicSpline>(), // roll
				std::make_shared<bitbots_splines::CubicSpline>(), // pitch
				std::make_shared<bitbots_splines::CubicSpline>()  // yaw
			),
			std::make_shared<bitbots_splines::PoseHandle>( // Trunk
				std::make_shared<bitbots_splines::CubicSpline>(), // x
				std::make_shared<bitbots_splines::CubicSpline>(), // y
				std::make_shared<bitbots_splines::CubicSpline>(), // z
				std::make_shared<bitbots_splines::CubicSpline>(), // roll
				std::make_shared<bitbots_splines::CubicSpline>(), // pitch
				std::make_shared<bitbots_splines::CubicSpline>()  // yaw
			)
		){
	}
} //bitbots_throw