#include "throws/throw_curves/beziercurve_throw.h"
#include "../../bitbots_splines_extension/include/spline/beziercurve.h"

namespace bitbots_throw{
	BeziercurveThrow::BeziercurveThrow()
		:ThrowCurve(
			std::make_shared<bitbots_splines::PoseHandle>( // Left Hand
				std::make_shared<bitbots_splines::Beziercurve>() // x
			   ,std::make_shared<bitbots_splines::Beziercurve>() // y
			   ,std::make_shared<bitbots_splines::Beziercurve>() // z
			   ,std::make_shared<bitbots_splines::Beziercurve>() // roll
			   ,std::make_shared<bitbots_splines::Beziercurve>() // pitch
			   ,std::make_shared<bitbots_splines::Beziercurve>() // yaw
			),
			std::make_shared<bitbots_splines::PoseHandle>( // Right Hand
				std::make_shared<bitbots_splines::Beziercurve>() // x
			   ,std::make_shared<bitbots_splines::Beziercurve>() // y
			   ,std::make_shared<bitbots_splines::Beziercurve>() // z
			   ,std::make_shared<bitbots_splines::Beziercurve>() // roll
			   ,std::make_shared<bitbots_splines::Beziercurve>() // pitch
			   ,std::make_shared<bitbots_splines::Beziercurve>() // yaw
			),
            std::make_shared<bitbots_splines::PoseHandle>( // Left Feet
                std::make_shared<bitbots_splines::Beziercurve>() // x
               ,std::make_shared<bitbots_splines::Beziercurve>() // y
               ,std::make_shared<bitbots_splines::Beziercurve>() // z
               ,std::make_shared<bitbots_splines::Beziercurve>() // roll
               ,std::make_shared<bitbots_splines::Beziercurve>() // pitch
               ,std::make_shared<bitbots_splines::Beziercurve>() // yaw
            ),
            std::make_shared<bitbots_splines::PoseHandle>( // Right Feet
                std::make_shared<bitbots_splines::Beziercurve>() // x
               ,std::make_shared<bitbots_splines::Beziercurve>() // y
               ,std::make_shared<bitbots_splines::Beziercurve>() // z
               ,std::make_shared<bitbots_splines::Beziercurve>() // roll
               ,std::make_shared<bitbots_splines::Beziercurve>() // pitch
               ,std::make_shared<bitbots_splines::Beziercurve>() // yaw
            )
		){
	}
} //bitbots_throw