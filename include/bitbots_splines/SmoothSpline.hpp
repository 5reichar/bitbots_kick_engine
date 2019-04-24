/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef SMOOTHSPLINE_HPP
#define LEPH_SMOOTHSPLINE_HPP

#include "Spline.hpp"

namespace bitbots_splines {

/**
 * SmoothSpline
 *
 * Implementation of 5th order polynomial
 * splines trajectory known to minimize jerk
 */
class SmoothSpline : public Spline
{
    public:

        using Spline::addPoint;
        /**
         * Add a new point with its time, position value,
         * velocity and acceleration
         */
        virtual void addPoint(double time,
                                double position, 
                                double velocity = 0.0,
                                double acceleration = 0.0);
        
        /**
         * Recompute splines interpolation model
         */
        virtual void computeSplines() override;
    
    protected:

        /**
         * Inherit
         * Load Points
         */
        virtual void importCallBack() override;    
        
    private:
        
        /**
         * Fit a polynom between 0 and t with given
         * pos, vel and acc initial and final conditions
         */
        Polynom polynomFit(double t, 
            double pos1, double vel1, double acc1,
            double pos2, double vel2, double acc2) const;
};

}

#endif

