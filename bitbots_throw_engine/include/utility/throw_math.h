#ifndef BITBOTS_THROW_THROW_MATH_H
#define BITBOTS_THROW_THROW_MATH_H

#include <cmath>

namespace bitbots_throw{
    class ThrowMath{
    public:
        double calculate_distance(Struct3d point){
            return sqrt(pow(point.x_, 2) + pow(point.y_, 2));
        }

        double sin_deg(double angle){
            return std::sin(angle*pi_/180);
        }

        double cos_deg(double angle){
            return std::cos(angle*pi_/180);
        }

    private:
        double pi_ = 3.14159265;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_MATH_H
