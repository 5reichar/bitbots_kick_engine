#include <algorithm>
#include "bitbots_splines/Beziercurve.hpp"

namespace bitbots_splines
{

double Beziercurve::pos(double t) const
{
    double ret = 0;
    uint32_t total_degree = _points.size() - 1;

    for (uint32_t degree = 0; degree <= total_degree; ++degree)
    {
        auto b = calc_generall_bernstein_polynom(_points.front().time, _points.back().time, degree, total_degree, t);
        ret += b * _points[degree].position;
    }

    return ret;
}

double Beziercurve::vel(double t) const
{
    double ret = 0;
    uint32_t total_degree = _points.size() - 1;

    for (uint32_t degree = 0; degree <= total_degree; ++degree)
    {
        ret += calc_generall_bernstein_polynom(_points.front().time, _points.back().time, degree, total_degree - 1, t) * (_points[degree + 1].position - _points[degree].position);
    }

    return total_degree * ret;
}

double Beziercurve::acc(double t) const
{
    double ret = 0;
    uint32_t total_degree = _points.size() - 1;

    for (uint32_t degree = 0; degree <= total_degree; ++degree)
    {
        ret += calc_generall_bernstein_polynom(_points.front().time, _points.back().time, degree, total_degree - 2, t) * (_points[degree + 2].position - 2 * _points[degree + 1].position + _points[degree].position);
    }

    return total_degree * (total_degree - 1) * ret;
}

double Beziercurve::jerk(double t) const
{
    double ret = 0;
    uint32_t total_degree = _points.size() - 1;

    for (uint32_t degree = 0; degree <= total_degree; ++degree)
    {
        ret += calc_generall_bernstein_polynom(_points.front().time, _points.back().time, degree, total_degree - 3, t) * (_points[degree + 3].position - 3 * _points[degree + 2].position + 3 * _points[degree + 1].position - _points[degree].position);
    }

    return total_degree * (total_degree - 1) * (total_degree - 2) * ret;
}

double Beziercurve::posMod(double t) const
{
    double ret = 0;
    uint32_t total_degree = _points.size() - 1;

    for (uint32_t degree = 0; degree <= total_degree; ++degree)
    {
        ret += calc_bernstein_polynom(degree, total_degree, t) * _points[degree].position;
    }

    return ret;
}

double Beziercurve::velMod(double t) const
{
    double ret = 0;
    uint32_t total_degree = _points.size() - 1;

    for (uint32_t degree = 0; degree <= total_degree; ++degree)
    {
        ret += calc_bernstein_polynom(degree, total_degree - 1, t) * (_points[degree + 1].position - _points[degree].position);
    }

    return total_degree * ret;
}

double Beziercurve::accMod(double t) const
{
    double ret = 0;
    uint32_t total_degree = _points.size() - 1;

    for (uint32_t degree = 0; degree <= total_degree; ++degree)
    {
        ret += calc_bernstein_polynom(degree, total_degree - 2, t) * (_points[degree + 2].position - 2 * _points[degree + 1].position + _points[degree].position);
    }

    return total_degree * (total_degree - 1) * ret;
}

double Beziercurve::jerkMod(double t) const
{
    double ret = 0;
    uint32_t total_degree = _points.size() - 1;

    for (uint32_t degree = 0; degree <= total_degree; ++degree)
    {
        ret += calc_bernstein_polynom(degree, total_degree - 3, t) * (_points[degree + 3].position - 3 * _points[degree + 2].position + 3 * _points[degree + 1].position - _points[degree].position);
    }

    return total_degree * (total_degree - 1) * (total_degree - 2) * ret;
}

void Beziercurve::addPointCallBack()
{
    std::sort(
        _points.begin(),
        _points.end(),
        [](const Point &p1, const Point &p2) -> bool {
            return p1.time < p2.time;
        });
}

double Beziercurve::calc_generall_bernstein_polynom(uint32_t intervall_start, uint32_t intervall_end, uint32_t degree, uint32_t total_degree, double time) const
{
    double p1 = calc_power((intervall_end - intervall_start), total_degree);
    double p2 = calc_binomial_coefficient(total_degree, degree);
    double p3 = calc_power((time - intervall_start), degree);
    double p4 = calc_power((intervall_end - time), (total_degree - degree));
    double ret = (p2 * p3 * p4) / p1;

    return ret;
}

double Beziercurve::calc_bernstein_polynom(uint32_t degree, uint32_t total_degree, double time) const
{
    return calc_binomial_coefficient(total_degree, degree) * calc_power(time, degree) * calc_power((1 - time), (total_degree - degree));
}

double Beziercurve::calc_binomial_coefficient(uint32_t n, uint32_t i) const
{
    if (i >= n || i < 0 || n < 0)
    {
        return 1;
    }

    auto dividend = calc_factorial(i, n);
    auto divisor = calc_factorial(2, n - i);

    return dividend / divisor;
}

uint32_t Beziercurve::calc_factorial(uint32_t start, uint32_t end) const
{
    uint32_t ret = 1;

    for (uint32_t f = (start > 0 ? start : 1); f <= end; ++f)
    {
        ret *= f;
    }

    return ret;
}

double Beziercurve::calc_power(double base, uint32_t exponent) const
{
    double ret = 1.0;

    for (uint32_t i = 1; i <= exponent; ++i)
    {
        ret *= base;
    }

    return ret;
}

} // namespace bitbots_splines