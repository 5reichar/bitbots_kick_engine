#ifndef  BITBOTS_THROW_THROW_STRUCT_H
#define  BITBOTS_THROW_THROW_STRUCT_H

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
    };

    struct Color{
        double red_;
        double green_;
        double blue_;

        color(double red, double green, double blue)
                :red_(red), green_(green), blue_(blue){
        }
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_STRUCT_H