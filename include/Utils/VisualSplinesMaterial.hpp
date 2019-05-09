#ifndef VISUALSPLINESMATERIAL_HPP
#define VISUALSPLINESMATERIAL_HPP

#include "bitbots_splines/Curve.hpp"

class VisualSplinesMaterial
{
public:
    VisualSplinesMaterial(bitbots_splines::Curve *curve_x, bitbots_splines::Curve *curve_y, bitbots_splines::Curve *curve_z);
    ~VisualSplinesMaterial();

    void add_point_to_x(double const time, double const position, double const velocity, double const acceleration);
    void add_point_to_y(double const time, double const position, double const velocity, double const acceleration);
    void add_point_to_z(double const time, double const position, double const velocity, double const acceleration);

    double get_position_from_x(double const time);
    double get_position_from_y(double const time);
    double get_position_from_z(double const time);

    std::vector<std::vector<bitbots_splines::Curve::Point>> get_points() const;

protected:
private:
    bitbots_splines::Curve *m_p_curve_x;
    bitbots_splines::Curve *m_p_curve_y;
    bitbots_splines::Curve *m_p_curve_z;
};

#endif