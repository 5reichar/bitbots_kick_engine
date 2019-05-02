#include <assert.h>
#include "Utils/VisualSplinesMaterial.hpp"

VisualSplinesMaterial::VisualSplinesMaterial(bitbots_splines::Spline * spline_x, bitbots_splines::Spline * spline_y, bitbots_splines::Spline * spline_z)
    : m_p_spline_x(spline_x),
        m_p_spline_y(spline_y),
        m_p_spline_z(spline_z)
{

}

VisualSplinesMaterial::~VisualSplinesMaterial()
{
    delete m_p_spline_x;
    delete m_p_spline_y;
    delete m_p_spline_z;
}

void VisualSplinesMaterial::add_point_to_x(double const time, double const position, double const velocity, double const acceleration)
{
    assert(m_p_spline_x);

    m_p_spline_x->addPoint(bitbots_splines::Spline::createPoint(time, position, velocity, acceleration));
}

void VisualSplinesMaterial::add_point_to_y(double const time, double const position, double const velocity, double const acceleration)
{
    assert(m_p_spline_y);
    
    m_p_spline_y->addPoint(bitbots_splines::Spline::createPoint(time, position, velocity, acceleration));
}

void VisualSplinesMaterial::add_point_to_z(double const time, double const position, double const velocity, double const acceleration)
{
    assert(m_p_spline_z);

    m_p_spline_z->addPoint(bitbots_splines::Spline::createPoint(time, position, velocity, acceleration));
}

double VisualSplinesMaterial::get_position_from_x(double const time)
{
    return m_p_spline_x == NULL ? 0.0 : m_p_spline_x->pos(time);
}

double VisualSplinesMaterial::get_position_from_y(double const time)
{
    return m_p_spline_y == NULL ? 0.0 : m_p_spline_y->pos(time);
}

double VisualSplinesMaterial::get_position_from_z(double const time)
{
    return m_p_spline_z == NULL ? 0.0 : m_p_spline_z->pos(time);
}