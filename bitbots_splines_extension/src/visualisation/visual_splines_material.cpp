#include <assert.h>
#include "visualisation/visual_splines_material.h"

VisualSplinesMaterial::VisualSplinesMaterial(bitbots_splines::Curve *curve_x, bitbots_splines::Curve *curve_y, bitbots_splines::Curve *curve_z)
    : m_p_curve_x(curve_x),
      m_p_curve_y(curve_y),
      m_p_curve_z(curve_z),
      m_d_curve_scale_x(1.0),
      m_d_curve_scale_y(1.0),
      m_d_curve_scale_z(1.0),
      m_e_curve_color(Color::white),
      m_str_curve_namespace("Curve"),
      m_uint_id(0)
{
}

VisualSplinesMaterial::~VisualSplinesMaterial()
{
    delete m_p_curve_x;
    delete m_p_curve_y;
    delete m_p_curve_z;
}

/*
 * Curve Attributes
 */

void VisualSplinesMaterial::set_scale(double curve_scale)
{
    set_scale(curve_scale, curve_scale, curve_scale);
}

void VisualSplinesMaterial::set_scale(double curve_scale_x, double curve_scale_y, double curve_scale_z)
{
    m_d_curve_scale_x = curve_scale_x;
    m_d_curve_scale_y = curve_scale_y;
    m_d_curve_scale_z = curve_scale_z;
}

double VisualSplinesMaterial::get_scale_x() const
{
    return m_d_curve_scale_x;
}

double VisualSplinesMaterial::get_scale_y() const
{
    return m_d_curve_scale_y;
}

double VisualSplinesMaterial::get_scale_z() const
{
    return m_d_curve_scale_z;
}

void VisualSplinesMaterial::set_color(Color curve_color)
{
    m_e_curve_color = curve_color;
}

Color VisualSplinesMaterial::get_color() const
{
    return m_e_curve_color;
}

void VisualSplinesMaterial::set_namspace(std::string curve_namespace)
{
    m_str_curve_namespace = curve_namespace;
}

std::string VisualSplinesMaterial::get_namspace() const
{
    return m_str_curve_namespace;
}

void VisualSplinesMaterial::set_id(uint32_t curve_id)
{
    m_uint_id = curve_id;
}

uint32_t VisualSplinesMaterial::get_id() const
{
    return m_uint_id;
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

    auto point = bitbots_splines::Curve::create_point(time, position, velocity, acceleration);
    curve->add_point(point);
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
    return m_p_curve_x == NULL ? 0.0 : m_p_curve_x->position(time);
}

double VisualSplinesMaterial::get_position_from_y(double const time)
{
    return m_p_curve_y == NULL ? 0.0 : m_p_curve_y->position(time);
}

double VisualSplinesMaterial::get_position_from_z(double const time)
{
    return m_p_curve_z == NULL ? 0.0 : m_p_curve_z->position(time);
}