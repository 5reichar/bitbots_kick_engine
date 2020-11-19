#ifndef BITBOTS_THROW_THROW_MATH_H
#define BITBOTS_THROW_THROW_MATH_H

#include <cmath>

namespace bitbots_throw{
    class ThrowMath{
    public:
        static double calculate_distance(Struct3d point){
            return sqrt(pow(point.x_, 2) + pow(point.y_, 2));
        }

        static double degree_to_radian(double angle_deg, double const pi){
            return angle_deg * pi / 180;
        }

        static double calculate_angle(Struct3d & point, double const & pi){
            auto angle = std::atan((point.y_ / point.x_));

            if(angle > pi){
                angle -= 2 * pi;
            }else if(angle < pi){
                angle += 2 * pi;
            }

            if((point.y_ < 0 && angle > 0) || (point.y_ > 0 && angle < 0)){
                angle *= -1;
            }

            return angle;
        }

        static Struct3dRPY calculate_throw_start_point(double const & throw_release_angle
                                                      ,double const & throw_zenith
                                                      ,double const & pi){
            Struct3dRPY throw_start_point;
            // From Isosceles triangle with the two angles with the same size are given as 90 - throw_angle
            auto hypotenuse = std::sqrt(2 * std::pow(throw_zenith, 2) * (1 - std::cos(2 * throw_release_angle)));
            auto opposite = std::sin(throw_release_angle) * hypotenuse;
            auto adjacent = std::cos(throw_release_angle) * hypotenuse;

            throw_start_point.x_ = -1 * adjacent;
            throw_start_point.y_ = 0.0;
            throw_start_point.z_ = throw_zenith - opposite;
            throw_start_point.roll_ = 0.0;
            throw_start_point.pitch_ = -2 * throw_release_angle - pi / 2;
            throw_start_point.yaw_ = 0.0;

            return throw_start_point;
        };

        static std::pair<double, double> calculate_foot_movement_for_rotate_robot(Struct3d const & rotation_goal, double const & foot_distance){
            //TODO: testing
            std::pair<double, double> return_value = {0.0, 0.0};

            auto x_2 = std::pow(rotation_goal.x_, 2);
            auto y_2 = std::pow(rotation_goal.y_, 2);
            auto temp_1 = std::sqrt(1 - (y_2 / (x_2 + y_2)));
            auto temp_2 = std::sqrt(1 - (y_2 / (2 * (x_2 + y_2))));

            return_value.first = 2 * foot_distance * (1 - temp_1) * temp_2;
            return_value.second = 2 * foot_distance * (1 - temp_1) * temp_1;

            return return_value;
        }

        static void calculate_positions_after_coordinate_rotation(double const & rotation_angle, double & x, double & y, bool const & rotate_left){
            double old_x = x;
            double old_y = y;

            if(rotate_left){
                x = old_x * std::cos(rotation_angle) + old_y * std::sin(rotation_angle);
                y = -1 * old_x * std::sin(rotation_angle) + old_y * std::cos(rotation_angle);
            }else{
                x = old_x * std::cos(rotation_angle) - old_y * std::sin(rotation_angle);
                y = old_x * std::sin(rotation_angle) + old_y * std::cos(rotation_angle);
            }
        }

    private:
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_MATH_H
