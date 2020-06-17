#ifndef  BITBOTS_THROW_STRUCT3D_H
#define  BITBOTS_THROW_STRUCT3D_H

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

    struct Struct3dRPY{
        double x_;
        double y_;
        double z_;
        double roll_;
        double pitch_;
        double yaw_;

        //////		Constructor
        Struct3dRPY(double x, double y, double z, double roll, double pitch, double yaw)
                : x_{x}, y_{y}, z_{z}, roll_{roll}, pitch_{pitch}, yaw_{yaw}{
        }

        Struct3dRPY() = default;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_STRUCT3D_H