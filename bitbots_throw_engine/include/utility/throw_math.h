#ifndef BITBOTS_THROW_THROW_MATH_H
#define BITBOTS_THROW_THROW_MATH_H

#include <cmath>

namespace bitbots_throw{
    class ThrowMath{
    public:
        static double calculate_distance(Struct3d point){
            return sqrt(pow(point.x_, 2) + pow(point.y_, 2));
        }

        double sin_deg(double angle){
            return std::sin(angle*pi_/180);
        }

        double cos_deg(double angle){
            return std::cos(angle*pi_/180);
        }


        static double calculate_angle(Struct3d & point){
            return atan((point.y_ / point.x_));
        }

        static Struct3d calculate_throw_release_point(double const & throw_release_angle
                                                      ,double const & arm_length
                                                      ,double const & throw_orientation_angle
                                                      ,double const & throw_zenith){
            ThrowMath throw_math;
            Struct3d throw_release_point;
            // From Isosceles triangle with the two angles with the same size are given as 90 - throw_angle
            auto hypotenuse = std::sqrt(2 * std::pow(arm_length, 2) * (1 - throw_math.cos_deg(2 * throw_release_angle)));
            auto opposite = throw_math.sin_deg(throw_release_angle) * hypotenuse;
            auto adjacent = throw_math.cos_deg(throw_release_angle) * hypotenuse;

            throw_release_point.x_ = throw_math.cos_deg(throw_orientation_angle) * adjacent;
            throw_release_point.y_ = throw_math.sin_deg(throw_orientation_angle) * adjacent;
            throw_release_point.z_ = throw_zenith - opposite;

            return throw_release_point;
        };

    private:
        double pi_ = 3.14159265;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_MATH_H
