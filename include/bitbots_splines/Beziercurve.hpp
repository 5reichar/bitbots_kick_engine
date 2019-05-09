#ifndef BEZIERCURVE_HPP
#define BEZIERCURVE_HPP

#include "Curve.hpp"

namespace bitbots_splines
{

class Beziercurve : public Curve
{
public:
    /**
         * Return spline interpolation
         * at given t. Compute spline value,
         * its first, second and third derivative
         */
    double pos(double t) const override;
    double vel(double t) const override;
    double acc(double t) const override;
    double jerk(double t) const override;

    /**
         * Return spline interpolation
         * value, first, second and third derivative
         * with given t bound between 0 and 1
         */
    double posMod(double t) const override;
    double velMod(double t) const override;
    double accMod(double t) const override;
    double jerkMod(double t) const override;

private:
    double calc_generall_bernstein_polynom(uint32_t intervall_start, uint32_t intervall_end, uint32_t degree, uint32_t total_degree, double const time) const;
    double calc_bernstein_polynom(uint32_t degree, uint32_t total_degree, double const time) const;
    double calc_binomial_coefficient(uint32_t n, uint32_t i) const;
    uint32_t calc_factorial(uint32_t start, uint32_t end) const;
    double calc_power(double base, uint32_t exponent) const;

    void addPointCallBack() override;
};

} // namespace bitbots_splines

#endif