#ifndef BITBOTS_SPLINES_EXTENSION_BEZIERCURVE_H
#define BITBOTS_SPLINES_EXTENSION_BEZIERCURVE_H

#include "curve.h"
#include <cstdint>

namespace bitbots_splines
{

class Beziercurve : public Curve
{
public:
    double position(double t) const override;
    double velocity(double t) const override;
    double acceleration(double t) const override;
    double jerk(double t) const override;

    double position_mod(double t) const override;
    double velocity_mod(double t) const override;
    double acceleration_mod(double t) const override;
    double jerk_mod(double t) const override;

private:
    double calc_bezier_curve(double const time, double (Beziercurve::*bernstein_func)(double, uint32_t, uint32_t) const) const;

    double calc_bernstein_polynomial(double const time, uint32_t const i, uint32_t const degree) const;
    double calc_bernstein_polynomial_mod(double const time, uint32_t const i, uint32_t const degree) const;

    double calc_bernstein_polynomial_first_derivate(double const time, uint32_t const i, uint32_t const degree) const;
    double calc_bernstein_polynomial_first_derivate_mod(double const time, uint32_t const i, uint32_t const degree) const;

    double calc_bernstein_polynomial_second_derivate(double const time, uint32_t const i, uint32_t const degree) const;
    double calc_bernstein_polynomial_second_derivate_mod(double const time, uint32_t const i, uint32_t const degree) const;

    double calc_bernstein_polynomial_third_derivate(double const time, uint32_t const i, uint32_t const degree) const;
    double calc_bernstein_polynomial_third_derivate_mod(double const time, uint32_t const i, uint32_t const degree) const;

    double calc_binomial_coefficient(uint32_t n, uint32_t i) const;
    uint32_t calc_factorial(uint32_t start, uint32_t end) const;

    void add_point_call_back() override;
};

} // namespace bitbots_splines

#endif