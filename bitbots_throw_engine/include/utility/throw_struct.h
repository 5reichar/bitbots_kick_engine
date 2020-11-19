#ifndef  BITBOTS_THROW_THROW_STRUCT_H
#define  BITBOTS_THROW_THROW_STRUCT_H

#include "tf2/LinearMath/Transform.h"

namespace bitbots_throw{
    struct Struct3d{
        double x_;
        double y_;
        double z_;

        //////		Constructor
        Struct3d(double x, double y, double z)
                : x_{x}, y_{y}, z_{z}{
        }

        Struct3d() = default;
    };

    struct Struct3dRPY : public Struct3d{
        double roll_;
        double pitch_;
        double yaw_;

        //////		Constructor
        Struct3dRPY(double x, double y, double z, double roll, double pitch, double yaw)
                : Struct3d(x, y, z), roll_{roll}, pitch_{pitch}, yaw_{yaw}{
        }

        Struct3dRPY(Struct3d position)
                : Struct3d(position), roll_{0.0}, pitch_{0.0}, yaw_{0.0}{
        }

        Struct3dRPY() = default;

        Struct3d operator=(Struct3d position){
            this->x_ = position.x_;
            this->y_ = position.y_;
            this->z_ = position.z_;

            return position;
        }

        Struct3dRPY operator+(Struct3dRPY value){
            return {this->x_ + value.x_, this->y_ + value.y_, this->z_ + value.z_
                    ,this->roll_ + value.roll_, this->pitch_ + value.pitch_, this->yaw_ + value.yaw_};
        }

        Struct3dRPY & operator+=(Struct3dRPY value){
            this->x_ += value.x_;
            this->y_ += value.y_;
            this->z_ += value.z_;
            this->roll_ += value.roll_;
            this->pitch_ += value.pitch_;
            this->yaw_ += value.yaw_;
            return *this;
        }

        Struct3dRPY operator/(double value){
            return {this->x_ / value, this->y_ / value, this->z_ / value
                   ,this->roll_ / value, this->pitch_ / value, this->yaw_ / value};
        }

        Struct3dRPY & operator/=(double value){
            this->x_ /= value;
            this->y_ /= value;
            this->z_ /= value;
            this->roll_ /= value;
            this->pitch_ /= value;
            this->yaw_ /= value;
            return *this;
        }
    };

    enum IKMode{
        arms_and_legs_separated = 0,
        hands_only_and_legs_separated
    };

    struct ThrowRequest{
        Struct3d ball_position_;
        Struct3d goal_position_;

        Struct3dRPY head_position_;
        Struct3dRPY left_hand_position_;
        Struct3dRPY right_hand_position_;
        Struct3dRPY left_feet_position_;
        Struct3dRPY right_feet_position_;
    };

    struct ThrowResponse{
        IKMode ik_mode_;
        tf2::Transform support_foot_to_left_hand_;
        tf2::Transform support_foot_to_right_hand_;
        tf2::Transform support_foot_to_left_foot_;
        tf2::Transform support_foot_to_right_foot_;
    };



    struct Color{
        double red_;
        double green_;
        double blue_;

        Color(double red, double green, double blue)
                :red_(red), green_(green), blue_(blue){
        }

        Color operator+(const Color & c) const{
            return {red_ + c.red_, green_ + c.green_, blue_ + c.blue_};
        }

        Color & operator+=(const Color & c){
            red_ += c.red_;
            green_ += c.green_;
            blue_ += c.blue_;
            return *this;
        }

        Color operator-(const Color & c) const{
            return {red_ - c.red_, green_ - c.green_, blue_ - c.blue_};
        }

        Color & operator-=(const Color & c){
            red_ -= c.red_;
            green_ -= c.green_;
            blue_ -= c.blue_;
            return *this;
        }

        Color operator*(const double & d) const{
            return {red_ * d, green_ * d, blue_ * d};
        }

        Color & operator*=(const double & d){
            red_ *= d;
            green_ *= d;
            blue_ *= d;
            return *this;
        }

        Color operator/(const double & d) const{
            return {red_ / d, green_ / d, blue_ / d};
        }

        Color & operator/=(const double & d){
            red_ /= d;
            green_ /= d;
            blue_ /= d;
            return *this;
        }
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_STRUCT_H