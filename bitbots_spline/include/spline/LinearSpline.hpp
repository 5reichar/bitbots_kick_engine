/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/

#ifndef LEPH_LINEARSPLINE_HPP
#define LEPH_LINEARSPLINE_HPP

#include "Spline.hpp"

namespace bitbots_splines
{

/**
 * LinearSpline
 *
 * Implementation of 3th order
 * polynomial splines
 */
class LinearSpline : public Spline
{
public:
    using Spline::addPoint;
    /**
         * Add a new point with its time and position value,
         */
    virtual void addPoint(double time, double position);

private:
    /**
         * Recompute splines interpolation model
         */
    virtual void computeSplines() override;
};

} // namespace bitbots_splines

#endif
