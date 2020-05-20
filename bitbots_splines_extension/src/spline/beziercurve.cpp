#include <algorithm>
#include <cmath>
#include "spline/beziercurve.h"

namespace bitbots_splines
{
/*
 *  Position in time t
 */

double Beziercurve::position(double t) const
{
    return calc_bezier_curve(t, &Beziercurve::calc_bernstein_polynomial);
}

double Beziercurve::position_mod(double t) const
{
    return calc_bezier_curve(t, &Beziercurve::calc_bernstein_polynomial_mod);
}

double Beziercurve::calc_bezier_curve(double const time, double (Beziercurve::*bernstein_func)(double, uint32_t, uint32_t) const) const
{
    double return_value = 0;
    uint32_t degree = this->points_.size() - 1;

    for (uint32_t i = 0; i <= degree; ++i)
    {
        return_value += (this->*bernstein_func)(time, i, degree) * this->points_[i].position_;
    }

    return return_value;
}

/*
 *  Velocity in time t
 */

double Beziercurve::velocity(double t) const
{
    return calc_bezier_curve_derivative(t, &Beziercurve::calc_bernstein_polynomial);
}

double Beziercurve::velocity_mod(double t) const
{
    return calc_bezier_curve_derivative(t, &Beziercurve::calc_bernstein_polynomial_mod);
}

double Beziercurve::calc_bezier_curve_derivative(double const time, double (Beziercurve::*bernstein_func)(double const, uint32_t const, uint32_t const) const) const
{
    double return_value = 0;
    uint32_t degree = this->points_.size() - 1;

    for (uint32_t i = 0; i <= degree; ++i)
    {
        return_value += (this->*bernstein_func)(time, i, degree - 1) * (this->points_[i + 1].position_ - this->points_[i].position_);
    }

    return degree * return_value;
}

/*
 *  Accelaration in time t
 */

double Beziercurve::acceleration(double t) const
{
    return calc_bezier_curve_second_derivative(t, &Beziercurve::calc_bernstein_polynomial);
}

double Beziercurve::acceleration_mod(double t) const
{
    return calc_bezier_curve_second_derivative(t, &Beziercurve::calc_bernstein_polynomial_mod);
}

double Beziercurve::calc_bezier_curve_second_derivative(double const time, double (Beziercurve::*bernstein_func)(double const, uint32_t const, uint32_t const) const) const
{
    double return_value = 0;
    uint32_t degree = this->points_.size() - 1;

    for (uint32_t i = 0; i <= degree; ++i)
    {
        return_value += (this->*bernstein_func)(time, i, degree - 2) * (this->points_[i + 2].position_ - 2 * this->points_[i + 1].position_ + this->points_[i].position_);
    }

    return degree * (degree - 1) * return_value;
}

/*
 *  Jerk in time t
 */

double Beziercurve::jerk(double t) const
{
    return calc_bezier_curve_third_derivative(t, &Beziercurve::calc_bernstein_polynomial);
}

double Beziercurve::jerk_mod(double t) const
{
    return calc_bezier_curve_third_derivative(t, &Beziercurve::calc_bernstein_polynomial_mod);
}

double Beziercurve::calc_bezier_curve_third_derivative(double const time, double (Beziercurve::*bernstein_func)(double const, uint32_t const, uint32_t const) const) const
{
    double return_value = 0;
    uint32_t degree = this->points_.size() - 1;

    for (uint32_t i = 0; i <= degree; ++i)
    {
        return_value += (this->*bernstein_func)(time, i, degree - 3) * (this->points_[i + 3].position_ - 3 * this->points_[i + 2].position_ + 3 * this->points_[i + 1].position_ - this->points_[i].position_);
    }

    return degree * (degree - 1) * (degree - 2) * return_value;
}

/*
 *  Bernstein polynomial
 */

double Beziercurve::calc_bernstein_polynomial(double const time, uint32_t const i, uint32_t const degree) const
{
    uint32_t intervall_start = this->points_.front().time_;
    uint32_t intervall_end = this->points_.back().time_;

    double p1 = pow((intervall_end - intervall_start), degree);
    double p2 = calc_binomial_coefficient(degree, i);
    double p3 = pow((time - intervall_start), i);
    double p4 = pow((intervall_end - time), (degree - i));

    return (p2 * p3 * p4) / p1;
}

double Beziercurve::calc_bernstein_polynomial_mod(double const time, uint32_t const i, uint32_t const degree) const
{
    return calc_binomial_coefficient(degree, i) * pow(time, i) * pow((1 - time), (degree - i));
}

/*
 *  Additional mathematical functions
 */

double Beziercurve::calc_binomial_coefficient(uint32_t n, uint32_t k) const
{
    if (k == 0 || n == k)
    {
        return 1;
    }

    auto dividend = calc_factorial(1, n);
    auto divisor = calc_factorial(1, k) * calc_factorial(1, n - k);

    return dividend / divisor;
}

uint32_t Beziercurve::calc_factorial(uint32_t start, uint32_t end) const
{
    uint32_t return_value = 1;

    for (uint32_t f = (start > 0 ? start : 1); f <= end; ++f)
    {
        return_value *= f;
    }

    return return_value;
}

/*
 *  CallBacks
 */

void Beziercurve::add_point_call_back()
{
    std::sort(
        points_.begin(),
        points_.end(),
        [](const Point &p1, const Point &p2) -> bool {
            return p1.time_ < p2.time_;
        });
}

} // namespace bitbots_splines