#include "throws/throw_curves/smooth_spline_throw.h"
#include "../../bitbots_splines_extension/include/spline/smooth_spline.h"

namespace bitbots_throw{
	SmoothSplineThrow::SmoothSplineThrow()
		:ThrowCurve(
			std::make_shared<bitbots_splines::PoseHandle>( // Left Hand
				std::make_shared<bitbots_splines::SmoothSpline>() // x
			   ,std::make_shared<bitbots_splines::SmoothSpline>() // y
			   ,std::make_shared<bitbots_splines::SmoothSpline>() // z
			   ,std::make_shared<bitbots_splines::SmoothSpline>() // roll
			   ,std::make_shared<bitbots_splines::SmoothSpline>() // pitch
			   ,std::make_shared<bitbots_splines::SmoothSpline>() // yaw
			),
			std::make_shared<bitbots_splines::PoseHandle>( // Right Hand
				std::make_shared<bitbots_splines::SmoothSpline>() // x
			   ,std::make_shared<bitbots_splines::SmoothSpline>() // y
			   ,std::make_shared<bitbots_splines::SmoothSpline>() // z
			   ,std::make_shared<bitbots_splines::SmoothSpline>() // roll
			   ,std::make_shared<bitbots_splines::SmoothSpline>() // pitch
			   ,std::make_shared<bitbots_splines::SmoothSpline>() // yaw
			),
            std::make_shared<bitbots_splines::PoseHandle>( // Left Feet
                std::make_shared<bitbots_splines::SmoothSpline>() // x
               ,std::make_shared<bitbots_splines::SmoothSpline>() // y
               ,std::make_shared<bitbots_splines::SmoothSpline>() // z
               ,std::make_shared<bitbots_splines::SmoothSpline>() // roll
               ,std::make_shared<bitbots_splines::SmoothSpline>() // pitch
               ,std::make_shared<bitbots_splines::SmoothSpline>() // yaw
            ),
            std::make_shared<bitbots_splines::PoseHandle>( // Right Feet
                std::make_shared<bitbots_splines::SmoothSpline>() // x
               ,std::make_shared<bitbots_splines::SmoothSpline>() // y
               ,std::make_shared<bitbots_splines::SmoothSpline>() // z
               ,std::make_shared<bitbots_splines::SmoothSpline>() // roll
               ,std::make_shared<bitbots_splines::SmoothSpline>() // pitch
               ,std::make_shared<bitbots_splines::SmoothSpline>() // yaw
            )
		){
	}
} //bitbots_throw