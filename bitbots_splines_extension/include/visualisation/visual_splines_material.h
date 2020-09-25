#ifndef BITBOTS_SPLINES_EXTENSION_VISUAL_SPLINES_MATERIAL_H
#define BITBOTS_SPLINES_EXTENSION_VISUAL_SPLINES_MATERIAL_H

#include <string>
#include "spline/curve.h"
#include "handle/pose_handle.h"

namespace bitbots_splines{
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
        VisualSplinesMaterial(std::shared_ptr<PoseHandle> & pose_handle);

        virtual void set_scale(double curve_scale);
        virtual void set_scale(double curve_x, double curve_y, double curve_z, double curve_roll, double curve_pitch, double curve_yaw);
        double get_scale_x() const;
        double get_scale_y() const;
        double get_scale_z() const;
        double get_scale_roll() const;
        double get_scale_pitch() const;
        double get_scale_yaw() const;

        void set_color(Color curve_color);
        Color get_color() const;

        void set_namspace(std::string curve_namespace);
        std::string get_namspace() const;

        void set_id(uint32_t curve_id);
        uint32_t get_id() const;

        void add_point_to_x(double const time, double const position, double const velocity, double const acceleration);
        void add_point_to_y(double const time, double const position, double const velocity, double const acceleration);
        void add_point_to_z(double const time, double const position, double const velocity, double const acceleration);
        void add_point_to_roll(double const time, double const position, double const velocity, double const acceleration);
        void add_point_to_pitch(double const time, double const position, double const velocity, double const acceleration);
        void add_point_to_yaw(double const time, double const position, double const velocity, double const acceleration);

        std::vector<std::vector<Curve::Point>> get_points() const;
        uint32_t get_number_of_points() const;

        double get_position_from_x(double const time);
        double get_position_from_y(double const time);
        double get_position_from_z(double const time);
        double get_position_from_roll(double const time);
        double get_position_from_pitch(double const time);
        double get_position_from_yaw(double const time);

    protected:
    private:
        void add_point(std::shared_ptr<Curve> curve, double const time, double const position, double const velocity, double const acceleration);

        std::shared_ptr<PoseHandle> sp_curve_;

        double d_curve_scale_x_;
        double d_curve_scale_y_;
        double d_curve_scale_z_;
        double d_curve_scale_roll_;
        double d_curve_scale_pitch_;
        double d_curve_scale_yaw_;

        Color m_e_curve_color;
        std::string m_str_curve_namespace;
        u_int32_t m_uint_id;
    };
}
#endif