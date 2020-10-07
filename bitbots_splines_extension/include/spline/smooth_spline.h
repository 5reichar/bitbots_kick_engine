/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_SPLINES_EXTENSION_SMOOTH_SPLINE_H
#define BITBOTS_SPLINES_EXTENSION_SMOOTH_SPLINE_H

#include "spline_base.h"

namespace bitbots_splines
{

/**
 * SmoothSpline
 *
 * Implementation of 5th order polynomial
 * splines trajectory known to minimize jerk
 */
class SmoothSpline : public SplineBase
{
public:
     /**
      * Recompute splines interpolation model
      */
    virtual void compute_splines() override;

protected:
     /**
      * Inherit
      * Load Points
      */
    virtual void import_call_back() override;

private:
     /**
      * Fit a polynom between 0 and t with given
      * pos, vel and acc initial and final conditions
      */
    Polynom polynom_fit(double t,
                        double pos1, double vel1, double acc1,
                        double pos2, double vel2, double acc2) const;
};

} // namespace bitbots_splines

#endif
