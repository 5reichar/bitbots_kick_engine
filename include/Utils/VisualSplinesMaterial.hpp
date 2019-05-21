#ifndef VISUALSPLINESMATERIAL_HPP
#define VISUALSPLINESMATERIAL_HPP

#include "bitbots_splines/Curve.hpp"

enum Color
{
    red,
    green,
    blue,
    yellow,
    white
};

class VisualSplinesMaterial
{
public:
    VisualSplinesMaterial(bitbots_splines::Curve *curve_x, bitbots_splines::Curve *curve_y, bitbots_splines::Curve *curve_z);
    ~VisualSplinesMaterial();

    virtual void set_scale(double curve_scale);
    virtual void set_scale(double curve_scale_x, double curve_scale_y, double curve_scale_z);
    double get_scale_x() const;
    double get_scale_y() const;
    double get_scale_z() const;

    void set_color(Color curve_color);
    Color get_color() const;

    void set_namspace(std::string curve_namespace);
    std::string get_namspace() const;

    void set_id(uint32_t curve_id);
    uint32_t get_id() const;

    void add_point_to_x(double const time, double const position, double const velocity, double const acceleration);
    void add_point_to_y(double const time, double const position, double const velocity, double const acceleration);
    void add_point_to_z(double const time, double const position, double const velocity, double const acceleration);

    std::vector<std::vector<bitbots_splines::Curve::Point>> get_points() const;
    uint32_t get_number_of_points() const;

    double get_position_from_x(double const time);
    double get_position_from_y(double const time);
    double get_position_from_z(double const time);

protected:
private:
    void add_point(bitbots_splines::Curve *curve, double const time, double const position, double const velocity, double const acceleration);

    bitbots_splines::Curve *m_p_curve_x;
    bitbots_splines::Curve *m_p_curve_y;
    bitbots_splines::Curve *m_p_curve_z;

    double m_d_curve_scale_x;
    double m_d_curve_scale_y;
    double m_d_curve_scale_z;

    Color m_e_curve_color;
    std::string m_str_curve_namespace;
    u_int32_t m_uint_id;
};

#endif