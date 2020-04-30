/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef SPLINE_HPP
#define SPLINE_HPP

#include <iostream>
#include "utils/polynom.h"
#include "curve.h"

namespace bitbots_splines
{

/**
 * Spline
 *
 * Generic one dimentional
 * polynomial spline generator
 */
class Spline : public Curve
{
public:
     /**
         * Internal spline part structure
         * with a polynom valid on an interval
         */
     struct Spline_t
     {
          Polynom polynom;
          double min;
          double max;
     };

     /**
         * Return spline interpolation
         * at given t. Compute spline value,
         * its first, second and third derivative
         */
     virtual double pos(double t) const override;
     virtual double vel(double t) const override;
     virtual double acc(double t) const override;
     virtual double jerk(double t) const override;

     /**
         * Return spline interpolation
         * value, first, second and third derivative
         * with given t bound between 0 and 1
         */
     virtual double pos_mod(double t) const override;
     virtual double vel_mod(double t) const override;
     virtual double acc_mod(double t) const override;
     virtual double jerk_mod(double t) const override;

     /**
         * Return minimum and maximum abscisse
         * value for which spline is defined
         */
     double min() const;
     double max() const;

     virtual void compute_splines() = 0;

     /**
         * Write and read splines data into given
         * iostream in ascii format
         */
     void export_data(std::ostream &os) const;
     void import_data(std::istream &is);

     /**
         * Return the number of internal polynom
         */
     size_t size() const;

     /**
         * Access to given by its index
         */
     const Spline_t &part(size_t index) const;

     /**
         * Add a part with given polynom
         * and min/max time range
         */
     void add_part(const Polynom &poly, double min, double max);

     /**
         * Replace this spline part with the
         * internal data of given spline
         */
     void copy_data(const Spline &sp);

protected:
     /**
         * Spline part container
         */
     std::vector<Spline_t> splines_;

     /**
         * Possible override callback
         * after importation
         */
     virtual void import_call_back();

     virtual void add_point_call_back() override;

private:
     /**
         * Return spline interpolation of given value and
         * used given polynom evaluation function
         * (member function pointer)
         */
     double interpolation(double x, double (Polynom::*func)(double) const) const;

     /**
         * Return interpolation with x
         * bound between 0 and 1
         */
     double interpolation_mod(double x, double (Polynom::*func)(double) const) const;
};

} // namespace bitbots_splines

#endif
