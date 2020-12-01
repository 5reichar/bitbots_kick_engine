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
