#ifndef BEZIERCURVE_HPP
#define BEZIERCURVE_HPP

#include "Curve.hpp"

namespace bitbots_splines
{

class Beziercurve : public Curve
{
public:
    double pos(double t) const override;
    double vel(double t) const override;
    double acc(double t) const override;
    double jerk(double t) const override;

    double posMod(double t) const override;
    double velMod(double t) const override;
    double accMod(double t) const override;
    double jerkMod(double t) const override;

private:
    double calc_bezier_curve(double const time, double (Beziercurve::*bernstein_func)(double, uint32_t, uint32_t) const) const;
    double calc_bezier_curve_derivative(double const time, double (Beziercurve::*bernstein_func)(double const, uint32_t const, uint32_t const) const) const;
    double calc_bezier_curve_second_derivative(double const time, double (Beziercurve::*bernstein_func)(double const, uint32_t const, uint32_t const) const) const;
    double calc_bezier_curve_third_derivative(double const time, double (Beziercurve::*bernstein_func)(double const, uint32_t const, uint32_t const) const) const;

    double calc_bernstein_polynomial(double const time, uint32_t const i, uint32_t const degree) const;
    double calc_bernstein_polynomial_mod(double const time, uint32_t const i, uint32_t const degree) const;

    double calc_binomial_coefficient(uint32_t n, uint32_t i) const;
    uint32_t calc_factorial(uint32_t start, uint32_t end) const;
    double calc_power(double base, uint32_t exponent) const;

    void addPointCallBack() override;
};

} // namespace bitbots_splines

#endif