#include <assert.h>
#include "Utils/VisualSplinesMaterial.hpp"

VisualSplinesMaterial::VisualSplinesMaterial(bitbots_splines::Curve *curve_x, bitbots_splines::Curve *curve_y, bitbots_splines::Curve *curve_z)
    : m_p_curve_x(curve_x),
      m_p_curve_y(curve_y),
      m_p_curve_z(curve_z)
{
}

VisualSplinesMaterial::~VisualSplinesMaterial()
{
    delete m_p_curve_x;
    delete m_p_curve_y;
    delete m_p_curve_z;
}

/*
 * Point handeling
 */

void VisualSplinesMaterial::add_point_to_x(double const time, double const position, double const velocity, double const acceleration)
{
    add_point(m_p_curve_x, time, position, velocity, acceleration);
}

void VisualSplinesMaterial::add_point_to_y(double const time, double const position, double const velocity, double const acceleration)
{
    add_point(m_p_curve_y, time, position, velocity, acceleration);
}

void VisualSplinesMaterial::add_point_to_z(double const time, double const position, double const velocity, double const acceleration)
{
    add_point(m_p_curve_z, time, position, velocity, acceleration);
}

void VisualSplinesMaterial::add_point(bitbots_splines::Curve *curve, double const time, double const position, double const velocity, double const acceleration)
{
    assert(curve);

    auto point = bitbots_splines::Curve::createPoint(time, position, velocity, acceleration);
    curve->addPoint(point);
}

std::vector<std::vector<bitbots_splines::Curve::Point>> VisualSplinesMaterial::get_points() const
{
    std::vector<std::vector<bitbots_splines::Curve::Point>> points;

    points.push_back(m_p_curve_x->points());
    points.push_back(m_p_curve_y->points());
    points.push_back(m_p_curve_z->points());

    return points;
}

uint32_t VisualSplinesMaterial::get_number_of_points() const
{
    return m_p_curve_x->points().size();
}

/*
 * Position handeling
 */

double VisualSplinesMaterial::get_position_from_x(double const time)
{
    return m_p_curve_x == NULL ? 0.0 : m_p_curve_x->pos(time);
}

double VisualSplinesMaterial::get_position_from_y(double const time)
{
    return m_p_curve_y == NULL ? 0.0 : m_p_curve_y->pos(time);
}

double VisualSplinesMaterial::get_position_from_z(double const time)
{
    return m_p_curve_z == NULL ? 0.0 : m_p_curve_z->pos(time);
}