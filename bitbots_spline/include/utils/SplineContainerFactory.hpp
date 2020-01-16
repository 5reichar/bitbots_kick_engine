#ifndef SPLINECONTAINERFACTORY_HPP
#define SPLINECONTAINERFACTORY_HPP

#include "utils/SplineContainer.hpp"

namespace bitbots_splines
{
    class SplineContainerFactory
    {
        public:
            static SplineContainer create_smooth_spline_container();
    };
} // namespace bitbots_splines

#endif