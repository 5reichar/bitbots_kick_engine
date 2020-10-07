/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/

#ifndef BITBOTS_SPLINES_EXTENSION_CUBIC_SPLINE_H
#define BITBOTS_SPLINES_EXTENSION_CUBIC_SPLINE_H

#include "spline_base.h"

namespace bitbots_splines
{

/**
 * CubicSpline
 *
 * Implementation of 3th order
 * polynomial splines
 */
class CubicSpline : public SplineBase
{
public:
     /**
      * Apply normal random noise on spline point
      * position and velocity of given standart deviation.
      * If updateBounds is true, extremum point (min and max time)
      * are also updated.
      */
    void random_noise(double stdDevPos, double stdDevVel, bool updateBounds);

     /**
      * Add between each two following interpolation
      * points a given number of point uniformaly
      * distributed. Cubic splines are splited.
      */
    void subdivide(unsigned int divider);

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
                        double pos1, double vel1,
                        double pos2, double vel2) const;
};

} // namespace bitbots_splines

#endif