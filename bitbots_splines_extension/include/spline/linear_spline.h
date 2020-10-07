/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/

#ifndef BITBOTS_SPLINES_EXTENSION_LINEAR_SPLINE_H
#define BITBOTS_SPLINES_EXTENSION_LINEAR_SPLINE_H

#include "spline_base.h"

namespace bitbots_splines
{

/**
 * LinearSpline
 *
 * Implementation of 3th order
 * polynomial splines
 */
class LinearSpline : public SplineBase
{
public:
private:
    /**
     * Recompute splines interpolation model
     */
    virtual void compute_splines() override;
};

} // namespace bitbots_splines

#endif
