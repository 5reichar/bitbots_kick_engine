#ifndef SPLINECONTAINERFACTORY_HPP
#define SPLINECONTAINERFACTORY_HPP

#include "utils/spline_container.hpp"

namespace bitbots_splines
{
    class SplineContainerFactory
    {
        public:
            static SplineContainer create_linear_spline_container();
            static SplineContainer create_cubic_spline_container();
            static SplineContainer create_smooth_spline_container();
            static SplineContainer create_beziercurve_container();
    };
} // namespace bitbots_splines

#endif