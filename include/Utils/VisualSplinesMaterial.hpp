#include "bitbots_splines/Spline.hpp"

class VisualSplinesMaterial
{
    public:
        VisualSplinesMaterial(bitbots_splines::Spline * spline_x, bitbots_splines::Spline * spline_y, bitbots_splines::Spline * spline_z);
        ~VisualSplinesMaterial();

        void add_point_to_x(double const time, double const position, double const velocity, double const acceleration);
        void add_point_to_y(double const time, double const position, double const velocity, double const acceleration);
        void add_point_to_z(double const time, double const position, double const velocity, double const acceleration);

        double get_position_from_x(double const time);
        double get_position_from_y(double const time);
        double get_position_from_z(double const time);

    protected:
    
    private:
        bitbots_splines::Spline * m_p_spline_x;
        bitbots_splines::Spline * m_p_spline_y;
        bitbots_splines::Spline * m_p_spline_z;
    
};