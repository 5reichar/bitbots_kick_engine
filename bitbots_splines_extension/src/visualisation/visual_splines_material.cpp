#include <assert.h>
#include "visualisation/visual_splines_material.h"

namespace bitbots_splines{
    VisualSplinesMaterial::VisualSplinesMaterial(std::shared_ptr<PoseHandle> & pose_handle)
            : sp_curve_(pose_handle),
              d_curve_scale_x_(1.0),
              d_curve_scale_y_(1.0),
              d_curve_scale_z_(1.0),
              d_curve_scale_roll_(1.0),
              d_curve_scale_pitch_(1.0),
              d_curve_scale_yaw_(1.0),
              m_e_curve_color(Color::white),
              m_str_curve_namespace("Curve"),
              m_uint_id(0)
    {
    }

/*
 * Curve Attributes
 */
    void VisualSplinesMaterial::set_scale(double curve_scale)
    {
        set_scale(curve_scale, curve_scale, curve_scale, curve_scale, curve_scale, curve_scale);
    }

    void VisualSplinesMaterial::set_scale(double curve_x, double curve_y, double curve_z, double curve_roll, double curve_pitch, double curve_yaw)
    {
        d_curve_scale_x_ = curve_x;
        d_curve_scale_y_ = curve_y;
        d_curve_scale_z_ = curve_z;
        d_curve_scale_roll_ = curve_roll;
        d_curve_scale_pitch_ = curve_pitch;
        d_curve_scale_yaw_ = curve_yaw;
    }

    double VisualSplinesMaterial::get_scale_x() const
    {
        return d_curve_scale_x_;
    }

    double VisualSplinesMaterial::get_scale_y() const
    {
        return d_curve_scale_y_;
    }

    double VisualSplinesMaterial::get_scale_z() const
    {
        return d_curve_scale_z_;
    }

    double VisualSplinesMaterial::get_scale_roll() const{
        return d_curve_scale_roll_;
    }

    double VisualSplinesMaterial::get_scale_pitch() const{
        return d_curve_scale_pitch_;
    }

    double VisualSplinesMaterial::get_scale_yaw() const{
        return d_curve_scale_yaw_;
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
        add_point(sp_curve_->x(), time, position, velocity, acceleration);
    }

    void VisualSplinesMaterial::add_point_to_y(double const time, double const position, double const velocity, double const acceleration)
    {
        add_point(sp_curve_->y(), time, position, velocity, acceleration);
    }

    void VisualSplinesMaterial::add_point_to_z(double const time, double const position, double const velocity, double const acceleration)
    {
        add_point(sp_curve_->z(), time, position, velocity, acceleration);
    }

    void VisualSplinesMaterial::add_point_to_roll(double const time, double const position, double const velocity, double const acceleration)
    {
        add_point(sp_curve_->roll(), time, position, velocity, acceleration);
    }

    void VisualSplinesMaterial::add_point_to_pitch(double const time, double const position, double const velocity, double const acceleration)
    {
        add_point(sp_curve_->pitch(), time, position, velocity, acceleration);
    }

    void VisualSplinesMaterial::add_point_to_yaw(double const time, double const position, double const velocity, double const acceleration)
    {
        add_point(sp_curve_->yaw(), time, position, velocity, acceleration);
    }

    void VisualSplinesMaterial::add_point(std::shared_ptr<Curve> curve, double const time, double const position, double const velocity, double const acceleration)
    {
        assert(curve);

        auto point = Curve::create_point(time, position, velocity, acceleration);
        curve->add_point(point);
    }

    std::vector<std::vector<Curve::Point>> VisualSplinesMaterial::get_points() const
    {
        std::vector<std::vector<Curve::Point>> points;

        points.push_back(sp_curve_->x()->points());
        points.push_back(sp_curve_->y()->points());
        points.push_back(sp_curve_->z()->points());
        points.push_back(sp_curve_->roll()->points());
        points.push_back(sp_curve_->pitch()->points());
        points.push_back(sp_curve_->yaw()->points());

        return points;
    }

    uint32_t VisualSplinesMaterial::get_number_of_points() const
    {
        return sp_curve_->x()->points().size();
    }

/*
 * Position handeling
 */

    double VisualSplinesMaterial::get_position_from_x(double const time)
    {
        return sp_curve_->x() == nullptr ? 0.0 : sp_curve_->x()->position(time);
    }

    double VisualSplinesMaterial::get_position_from_y(double const time)
    {
        return sp_curve_->y() == nullptr ? 0.0 : sp_curve_->y()->position(time);
    }

    double VisualSplinesMaterial::get_position_from_z(double const time)
    {
        return sp_curve_->z() == nullptr ? 0.0 : sp_curve_->z()->position(time);
    }

    double VisualSplinesMaterial::get_position_from_roll(const double time)
    {
        return sp_curve_->roll() == nullptr ? 0.0 : sp_curve_->roll()->position(time);
    }

    double VisualSplinesMaterial::get_position_from_pitch(double const time)
    {
        return sp_curve_->pitch() == nullptr ? 0.0 : sp_curve_->pitch()->position(time);
    }

    double VisualSplinesMaterial::get_position_from_yaw(double const time)
    {
        return sp_curve_->yaw() == nullptr ? 0.0 : sp_curve_->yaw()->position(time);
    }
}