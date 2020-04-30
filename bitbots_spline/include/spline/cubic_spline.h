/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/

#ifndef LEPH_CUBICSPLINE_HPP
#define LEPH_CUBICSPLINE_HPP

#include "spline.h"

namespace bitbots_splines
{

/**
 * CubicSpline
 *
 * Implementation of 3th order
 * polynomial splines
 */
class CubicSpline : public Spline
{
public:
    using Spline::add_point;
     /**
      * Add a new point with its time, position value,
      * and velocity
      */
    virtual void add_point(double time, double position, double velocity = 0.0);

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